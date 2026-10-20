#include "ShutterNotifyStage.h"

#include <utils/Timers.h>

#include "Frame.h"
#include "mochacamera/ports/ResultSink.h"

namespace android {

ShutterNotifyStage::ShutterNotifyStage(mocha::camera::ResultSink *s)
    : sink(s) {}

void ShutterNotifyStage::process(mocha::camera::Frame &frame) {
    frame.tShutter = systemTime();
    sink->shutter(frame.frameNumber, frame.tShutter);
}

} /* namespace android */
