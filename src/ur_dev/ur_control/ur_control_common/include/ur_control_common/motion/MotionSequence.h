#ifndef UR_CONTROL_COMMON_MOTIONSEQUENCE_H_
#define UR_CONTROL_COMMON_MOTIONSEQUENCE_H_

//System headers
#include <cstdint>
#include <string>
#include <unordered_map>

//Other libraries headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

class MotionSequence : public NonCopyable, public NonMoveable { 
public:
  MotionSequence(const std::string& name, int32_t id);
  virtual ~MotionSequence() noexcept = default;

  virtual void start(const UscriptsBatchDoneCb& cb) = 0;
  virtual void gracefulStop(const UscriptsBatchDoneCb& cb) = 0;
  virtual void recover(const UscriptsBatchDoneCb& cb) = 0;

  void abort(const UscriptsBatchDoneCb& cb);
  void setDispatchUscriptsAsyncCb(const DispatchUscriptsAsyncCb& cb);

  int32_t getId() const;
  std::string getName() const;

protected:
  using UrScriptHeaders = std::unordered_map<std::string, UrScriptPayload>;
  UrScriptHeaders urScriptHeaders;
  DispatchUscriptsAsyncCb dispatchUscriptsAsyncCb;

private:
  std::string _name;
  int32_t _id { };
};

#endif /* UR_CONTROL_COMMON_MOTIONSEQUENCE_H_ */
