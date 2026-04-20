#ifndef HAL_PIPELINE_PIPELINE_H
#define HAL_PIPELINE_PIPELINE_H

#include <memory>
#include <vector>

#include "PipelineStage.h"

namespace android {

struct PipelineContext;

/* Ordered collection of stages executed on one thread against a
 * PipelineContext. Stages run in insertion order; if a stage sets
 * context.errorCode to non-zero, subsequent stages are skipped
 * unless they report alwaysRun()==true. */
class Pipeline {
public:
    Pipeline() = default;

    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    void appendStage(std::unique_ptr<PipelineStage> stage);

    void run(PipelineContext &context);

private:
    std::vector<std::unique_ptr<PipelineStage>> stages;
};

} /* namespace android */

#endif /* HAL_PIPELINE_PIPELINE_H */
