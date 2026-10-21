#!/bin/bash
#
# Copyright (C) 2026 Artem Bambalov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The core's host tests, on a Linux host (the build server), under
# ThreadSanitizer. Run from the repository root:
#
#   core/tests/run_host_tests.sh [out-dir]
#
# out-dir defaults to ../../out/camera-host-tests -- the Android tree's
# out/ when the repository is hardware/camera in it. Needs clang++ and
# the host's jsoncpp. Also checks the core's one build invariant: no
# file under core/ includes camera3.h or CameraMetadata.

set -u
OUT="${1:-../../out/camera-host-tests}"
mkdir -p "$OUT"

CXX="${CXX:-clang++}"
CXXFLAGS=(-std=c++20 -Wall -Wextra -Werror -fsanitize=thread -g -pthread
          -DV4L2DEVICE_BUF_COUNT=8
          -Icore/tests/stub -Icore -Icore/include -Icore/base -Icore/engine
          -Icore/engines -Icore/pipeline -Icore/pipeline/stages -Icore/ipa
          -Icore/isp -Icore/util -Icore/session)

failed=0

if grep -rnE --exclude-dir=tests '#include.*(camera3\.h|CameraMetadata|camera_metadata\.h)' core; then
    echo "FAIL invariant: the core includes Camera3"
    failed=1
else
    echo "ok   invariant: the core includes no Camera3"
fi

# name, then its sources, then -- and the libraries it links
run() {
    local name="$1"; shift
    local srcs=() libs=()
    while [ $# -gt 0 ] && [ "$1" != -- ]; do srcs+=("$1"); shift; done
    [ $# -gt 0 ] && shift
    libs=("$@")
    if ! "$CXX" "${CXXFLAGS[@]}" "${srcs[@]}" -o "$OUT/$name" "${libs[@]}"; then
        echo "FAIL $name (build)"
        failed=1
        return
    fi
    if timeout 120 "$OUT/$name"; then
        echo "ok   $name"
    else
        echo "FAIL $name"
        failed=1
    fi
}

TUNING=(core/sensor/SensorTuning.cpp core/sensor/Pwl.cpp)

run job_queue_test       core/tests/job_queue_test.cpp
run engine_test          core/tests/engine_test.cpp core/base/EventFd.cpp
run permit_pool_test     core/tests/permit_pool_test.cpp core/base/EventFd.cpp
run frame_test           core/tests/frame_test.cpp core/pipeline/Frame.cpp \
                         core/pipeline/FrameRegistry.cpp core/base/EventFd.cpp
run capture_size_test   core/tests/capture_size_test.cpp core/session/CaptureSize.cpp
run delayed_controls_test core/tests/delayed_controls_test.cpp \
                         core/sensor/DelayedControls.cpp
run result_engine_test   core/tests/result_engine_test.cpp \
                         core/engines/ResultEngine.cpp \
                         core/pipeline/stages/ResultDispatchStage.cpp \
                         core/pipeline/Frame.cpp core/pipeline/FrameRegistry.cpp \
                         core/3a/ThreeAReport.cpp core/3a/AutoFocusController.cpp \
                         core/sensor/DelayedControls.cpp core/base/EventFd.cpp \
                         "${TUNING[@]}" -- -ljsoncpp
run three_a_test         core/tests/three_a_test.cpp \
                         core/3a/AutoExposureController.cpp \
                         core/3a/AutoFocusController.cpp \
                         core/3a/GrayWorldAwbController.cpp \
                         core/3a/ThreeAReport.cpp core/sensor/DelayedControls.cpp \
                         "${TUNING[@]}" -- -ljsoncpp

exit $failed
