#ifndef HAL_PIPELINE_PIPELINE_STAGE_H
#define HAL_PIPELINE_PIPELINE_STAGE_H

namespace mocha::camera { struct Frame; }

namespace android {


/* Abstract work unit executed by a Pipeline against a frame.
 *
 * A stage fails context.status to abort the frame; the
 * Pipeline then skips remaining non-alwaysRun stages. alwaysRun()
 * reports true for stages that must execute even in the error path
 * (primarily ResultDispatchStage, which must always signal the
 * framework). */
class PipelineStage {
public:
    virtual ~PipelineStage() = default;

    virtual const char *name() const = 0;
    virtual void process(mocha::camera::Frame &context) = 0;
    virtual bool alwaysRun() const { return false; }
};

} /* namespace android */

#endif /* HAL_PIPELINE_PIPELINE_STAGE_H */
