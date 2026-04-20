#include "Pipeline.h"

#include <utility>

#include "PipelineContext.h"

namespace android {

void Pipeline::appendStage(std::unique_ptr<PipelineStage> stage) {
    stages.push_back(std::move(stage));
}

void Pipeline::run(PipelineContext &context) {
    for (auto &stage : stages) {
        if (context.errorCode && !stage->alwaysRun()) continue;
        stage->process(context);
    }
}

} /* namespace android */
