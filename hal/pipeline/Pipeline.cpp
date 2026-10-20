#include "Pipeline.h"

#include <utility>

#include "Frame.h"

namespace android {

void Pipeline::appendStage(std::unique_ptr<PipelineStage> stage) {
    stages.push_back(std::move(stage));
}

void Pipeline::run(mocha::camera::Frame &context) {
    for (auto &stage : stages) {
        if (!context.status.ok() && !stage->alwaysRun()) continue;
        stage->process(context);
    }
}

} /* namespace android */
