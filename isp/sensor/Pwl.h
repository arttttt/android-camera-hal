#ifndef ISP_SENSOR_PWL_H
#define ISP_SENSOR_PWL_H

#include <stddef.h>
#include <vector>

namespace android {

/* Piecewise-linear 1-D map over (x, y) control points stored in
 * x-ascending order. `eval(x)` linearly interpolates between
 * bracketing points and clamps to the first / last point's y for
 * inputs outside the domain (flat extrapolation — the older
 * GrayLineSoftClamp ships explicit slope-before / slope-after
 * extensions, but Bayesian-AWB priors and CT curves want bounded
 * extrapolation, not extended).
 *
 * Used as a tuning-data primitive for the AWB-Bayes path: priors
 * (CT → log-likelihood, one PWL per calibrated lux level), CT
 * curves (CT → R/G or B/G ratio at calibration illuminants),
 * and any future single-axis tuning lookups. */
class Pwl {
public:
    Pwl();

    void   clear();

    /* Caller adds points in strict x-ascending order. No sort, no
     * dedup — the tuning JSON is the source of truth and produces
     * valid sequences by convention; out-of-order points produce
     * undefined eval. */
    void   addPoint(float x, float y);

    bool   empty() const { return points.empty(); }
    size_t size()  const { return points.size(); }

    float  minX() const;
    float  maxX() const;

    /* Linear interpolation between bracketing points. Out-of-range
     * inputs clamp to the first / last point's y. */
    float  eval(float x) const;

    /* Per-point read access for diagnostic logging / tooling. */
    float  xAt(size_t i) const { return points[i].x; }
    float  yAt(size_t i) const { return points[i].y; }

private:
    struct Point {
        float x;
        float y;
    };
    std::vector<Point> points;
};

} /* namespace android */

#endif /* ISP_SENSOR_PWL_H */
