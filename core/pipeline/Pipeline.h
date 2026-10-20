#ifndef HAL_PIPELINE_PIPELINE_H
#define HAL_PIPELINE_PIPELINE_H

#include <memory>
#include <vector>

#include "PipelineStage.h"

namespace mocha::camera { struct Frame; }

namespace android {


/* Ordered collection of stages executed on one thread against a
 * frame. Stages run in insertion order; once the frame's
 * status is no longer Ok (a stage failed it, or it was cancelled),
 * subsequent stages are skipped unless they report alwaysRun()==true. */
class Pipeline {
public:
    Pipeline() = default;

    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    void appendStage(std::unique_ptr<PipelineStage> stage);

    void run(mocha::camera::Frame &context);

private:
    std::vector<std::unique_ptr<PipelineStage>> stages;
};

} /* namespace android */

#endif /* HAL_PIPELINE_PIPELINE_H */
