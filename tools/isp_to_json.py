#!/usr/bin/env python3
"""Convert NVIDIA camera-override (.isp) files to HAL tuning JSON.

The .isp format is NVIDIA's bespoke `namespace.path[idx].sub = value;`
syntax with TRUE/FALSE bools, numeric scalars, and `{a, b, c}` tuples
that occasionally span multiple lines without a closing brace. Parser
is intentionally lenient — if an assignment's rhs doesn't close its
`{`, we treat the next statement-starting line as an implicit `}`.

Matrix layout note — `colorCorrection.Set[i].ccMatrix` (and the
top-level `colorCorrection.srgbMatrix`) is stored by NVIDIA as three
rows, where each row i is the contribution of INPUT channel i to
{R_out, G_out, B_out}. Our demosaic shader expects the opposite
convention (row i = OUTPUT-channel i's coefficients over input).
We preserve NVIDIA's layout verbatim in the JSON — the transpose is
applied by SensorTuning::ccmForCctQ10 at consumer time. Feeding the
matrix to the shader unchanged produces a neutral-grey-goes-to-magenta
failure mode (green channel cross-terms balloon).

Output:
    { "schema_version": 1,
      "sensor": { parsed-from-header-comments },
      "module": { externally-supplied via --module JSON },
      "active":   { keys that the HAL currently consumes },
      "reserved": { everything else preserved from the .isp } }

Dispatch rule: every parsed key whose path starts with one of
ACTIVE_PREFIXES goes under `active`; everything else under `reserved`.
Extend ACTIVE_PREFIXES when a new consumer lands in the HAL.
"""

import argparse
import json
import re
import sys
from collections import OrderedDict

ACTIVE_PREFIXES = (
    "af.",
    "colorCorrection.",
    "opticalBlack.",
    "mwbCCT.",
    # Fusion-AWB calibration — the linear U↔CCT map, its U clamp,
    # and the per-frame stats / smoothing thresholds BasicIpa uses
    # when filtering patches and damping its gain EMA. The HAL
    # consumes these to estimate scene CCT from the gray-world gains
    # NeonStatsEncoder produces, then picks / blends CcmSets by cctK.
    "awb.v4.UtoCCT",
    "awb.v4.CCTtoU",
    "awb.v4.LowU",
    "awb.v4.HighU",
    "awb.v4.CStatsMinThreshold",
    "awb.v4.CStatsDarkThreshold",
    "awb.v4.SmoothingWpTrackingFraction",
)

# Line that starts a new assignment: `<path> = ...`. Path may contain
# dots and `[N]` indexing.
ASSIGN_START = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_.\[\]]*)\s*=\s*(.*)$")
NUMBER = re.compile(r"^-?\d+(?:\.\d+)?$")


def parse_scalar(s):
    s = s.strip()
    if not s:
        return None
    up = s.upper()
    if up == "TRUE":
        return True
    if up == "FALSE":
        return False
    if s.startswith('"') and s.endswith('"'):
        return s[1:-1]
    if NUMBER.match(s):
        return float(s) if "." in s else int(s)
    return s  # fallback: bare identifier / unusual token


def parse_rhs(rhs):
    """Interpret the right-hand side of an assignment. Returns a scalar
    or list (possibly nested)."""
    rhs = rhs.strip().rstrip(";").strip()
    if not rhs.startswith("{"):
        return parse_scalar(rhs)

    # Tuple: strip braces (both may be absent/mismatched), split on commas.
    # NVIDIA files sometimes use `(x, y}` for 2-tuples — treat `(` as `{`.
    body = rhs
    body = body.replace("(", "{", 1) if body.startswith("(") else body
    body = body.lstrip("{").rstrip("}").strip()
    if not body:
        return []

    # Nested tuples (rare but possible): e.g. `{{a, b}, {c, d}}`.
    if "{" in body:
        # Simple depth tracker — split top-level commas only.
        parts, depth, cur = [], 0, []
        for ch in body:
            if ch == "{":
                depth += 1
                cur.append(ch)
            elif ch == "}":
                depth -= 1
                cur.append(ch)
            elif ch == "," and depth == 0:
                parts.append("".join(cur).strip())
                cur = []
            else:
                cur.append(ch)
        if cur:
            parts.append("".join(cur).strip())
        return [parse_rhs(p) for p in parts]

    # Flat tuple — comma-separated scalars, possibly with stray whitespace/
    # newlines. Split and parse.
    return [parse_scalar(tok) for tok in body.split(",") if tok.strip()]


def normalize_lines(text):
    """Rejoin LHS-on-line-N / `= value;`-on-line-N+1 splits. The stock .isp
    files do this freely for `colorCorrection.*` and similar sections —
    presumably a legacy linebreak from the tuning-tool export."""
    lines = text.splitlines()
    out = []
    i = 0
    while i < len(lines):
        cur = lines[i]
        stripped = cur.split("#", 1)[0].strip()
        if stripped and "=" not in stripped and i + 1 < len(lines):
            nxt = lines[i + 1].lstrip()
            if nxt.startswith("="):
                out.append(cur.rstrip() + " " + nxt)
                i += 2
                continue
        out.append(cur)
        i += 1
    return "\n".join(out)


def extract_statements(text):
    """Split input into logical statements. Each statement is (path, rhs_str).
    Continuation lines inside an unclosed `{` are joined; the tuple is closed
    synthetically when the next assignment begins."""
    text = normalize_lines(text)
    lines = text.splitlines()
    statements = []
    i = 0
    while i < len(lines):
        raw = lines[i]
        # strip NVIDIA-style comments (#* and leading #)
        stripped = raw.split("#", 1)[0]
        if not stripped.strip():
            i += 1
            continue
        m = ASSIGN_START.match(stripped)
        if not m:
            i += 1
            continue
        path, rhs = m.group(1), m.group(2)
        # Does the rhs open `{` without closing on the same line?
        if "{" in rhs and rhs.count("{") > rhs.count("}"):
            j = i + 1
            while j < len(lines):
                look = lines[j].split("#", 1)[0]
                if ASSIGN_START.match(look):
                    # Next statement begins — implicitly close the tuple.
                    rhs = rhs.rstrip() + " }"
                    break
                rhs += " " + look.strip()
                if rhs.count("{") <= rhs.count("}"):
                    j += 1
                    break
                j += 1
            else:
                rhs = rhs.rstrip() + " }"
            i = j
        else:
            i += 1
        statements.append((path, rhs))
    return statements


# Split `a.b[0].c.d[1]` into [("a",None),("b",0),("c",None),("d",1)].
PATH_PART = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)(?:\[(\d+)\])?")


def split_path(path):
    out = []
    for m in PATH_PART.finditer(path):
        name = m.group(1)
        idx = m.group(2)
        out.append((name, int(idx) if idx is not None else None))
    return out


def insert(tree, path_parts, value):
    """Insert value into a nested dict structure per the parsed path.
    Indexed segments store at stringified-int keys so that `foo.bar[0]`
    and `foo.bar.baz` can coexist under the same parent — NVIDIA's
    format uses this hybrid freely (e.g. `Chroma.Enable` + `Chroma[0].Gain`).
    Pure `0..N-1` key sets are converted back to JSON arrays in
    normalize_arrays() below."""
    cur = tree
    for k, (name, idx) in enumerate(path_parts):
        last = k == len(path_parts) - 1
        # descend into `name`
        nxt = cur.get(name)
        if not isinstance(nxt, (dict, OrderedDict)):
            nxt = OrderedDict()
            cur[name] = nxt
        if idx is None:
            if last:
                cur[name] = value
            else:
                cur = nxt
        else:
            key = str(idx)
            if last:
                nxt[key] = value
            else:
                child = nxt.get(key)
                if not isinstance(child, (dict, OrderedDict)):
                    child = OrderedDict()
                    nxt[key] = child
                cur = child


def normalize_arrays(node):
    """Post-pass: convert dicts whose keys are exactly 0..N-1 (stringified)
    into JSON arrays. Hybrid dicts (mix of names and numeric strings) stay
    as dicts, preserving both facets."""
    if isinstance(node, (dict, OrderedDict)):
        for k in list(node.keys()):
            node[k] = normalize_arrays(node[k])
        keys = list(node.keys())
        if keys and all(isinstance(k, str) and k.isdigit() for k in keys):
            nums = sorted(int(k) for k in keys)
            if nums == list(range(len(nums))):
                return [node[str(i)] for i in range(len(nums))]
        return node
    if isinstance(node, list):
        return [normalize_arrays(v) for v in node]
    return node


def parse_header(text):
    """Extract sensor/integrator/version from the `#* Key: Value` header."""
    meta = OrderedDict()
    for line in text.splitlines():
        if not line.startswith("#*"):
            break
        body = line.lstrip("#*").strip()
        if ":" in body:
            k, _, v = body.partition(":")
            meta[k.strip().lower()] = v.strip()
    result = OrderedDict()
    if "sensor" in meta:
        result["name"] = meta["sensor"]
    if "integrator" in meta:
        result["integrator"] = meta["integrator"]
    # nvidia_version from header is often stale (copy-paste leftover); the
    # filename is the authoritative version.
    if "version" in meta:
        result["nvidia_header_version"] = meta["version"]
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("infile")
    ap.add_argument("outfile")
    ap.add_argument(
        "--nvidia-version",
        help="authoritative version string (from filename); overrides header",
    )
    ap.add_argument(
        "--module",
        help="path to JSON with the `module` hardware block to merge in "
        "(physical_size_mm, focal_length_mm, etc. — data that's not in the .isp)",
    )
    args = ap.parse_args()

    with open(args.infile) as f:
        text = f.read()

    sensor = parse_header(text)
    sensor["source_file"] = args.infile.rsplit("/", 1)[-1]
    if args.nvidia_version:
        sensor["nvidia_version"] = args.nvidia_version

    active = OrderedDict()
    reserved = OrderedDict()
    for path, rhs in extract_statements(text):
        value = parse_rhs(rhs)
        target = active if path.startswith(ACTIVE_PREFIXES) else reserved
        try:
            insert(target, split_path(path), value)
        except Exception as e:
            sys.stderr.write("FAIL at `%s = %s`: %s\n" % (path, rhs[:60], e))
            raise

    out = OrderedDict()
    out["schema_version"] = 1
    out["sensor"] = sensor
    if args.module:
        with open(args.module) as f:
            out["module"] = json.load(f, object_pairs_hook=OrderedDict)
    out["active"] = normalize_arrays(active)
    out["_note"] = (
        "`active` — consumed by the current HAL; hardcodes in code are "
        "replaced with reads from here. `reserved` — preserved verbatim "
        "from the stock NVIDIA .isp, not consumed yet; keys migrate to "
        "`active` as the corresponding shader stages ship."
    )
    out["reserved"] = normalize_arrays(reserved)

    with open(args.outfile, "w") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
        f.write("\n")


if __name__ == "__main__":
    main()
