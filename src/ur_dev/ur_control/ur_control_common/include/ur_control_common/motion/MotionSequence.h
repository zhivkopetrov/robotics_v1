#ifndef UR_CONTROL_COMMON_MOTIONSEQUENCE_H_
#define UR_CONTROL_COMMON_MOTIONSEQUENCE_H_

//System headers
#include <cstdint>
#include <string>
#include <memory>

//Other libraries headers
#include "urscript_common/urscript/UrScriptBuilder.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"

//Forward declarations

class MotionSequence : public NonCopyable, public NonMoveable { 
public:
  MotionSequence(
    const std::string& name, int32_t id, 
    std::shared_ptr<UrScriptBuilder> urScriptBuilder);
  virtual ~MotionSequence() noexcept = default;

  virtual void start(const UrscriptsBatchDoneCb& cb) = 0;
  virtual void gracefulStop(const UrscriptsBatchDoneCb& cb) = 0;
  virtual void recover(const UrscriptsBatchDoneCb& cb) = 0;
  virtual ErrorCode setTransportStrategy(int32_t strategyId) = 0;

  void abort(const UrscriptsBatchDoneCb& cb);
  void setDispatchUscriptsAsyncCb(const DispatchUscriptsAsyncCb& cb);

  int32_t getId() const;
  std::string getName() const;

protected:
  UrScriptPayload constructUrScript(
    std::string_view methodName, 
    UrScriptCommandContainer& commandContainer) const;

  DispatchUscriptsAsyncCb dispatchUscriptsAsyncCb;

private:
  std::string _name;
  int32_t _id { };
  const std::shared_ptr<UrScriptBuilder> _urScriptBuilder;
};

#endif /* UR_CONTROL_COMMON_MOTIONSEQUENCE_H_ */
