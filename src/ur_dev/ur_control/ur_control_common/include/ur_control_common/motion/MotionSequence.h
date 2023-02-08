#ifndef UR_CONTROL_COMMON_MOTIONSEQUENCE_H_
#define UR_CONTROL_COMMON_MOTIONSEQUENCE_H_

//System headers
#include <cstdint>
#include <string>
#include <queue>
#include <unordered_map>
#include <functional>
#include <any>

//Other libraries headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

using UrScriptHeaders = std::unordered_map<std::string, UrScriptPayload>;

class MotionSequence : public NonCopyable, public NonMoveable { 
public:
  MotionSequence(
    const std::string& name, int32_t id, UrScriptHeaders&& headers);
  virtual ~MotionSequence() noexcept = default;

  virtual ErrorCode init(const std::any& cfg) = 0;
  virtual void start(const MotionCommandBatchDoneCb& cb) = 0;
  virtual void gracefulStop(const MotionCommandBatchDoneCb& cb) = 0;
  virtual void recover(const MotionCommandBatchDoneCb& cb) = 0;

  void abort(const MotionCommandBatchDoneCb& cb);
  void setDispatchMotionsAsyncCb(const DispatchMotionsAsyncCb& cb);

  int32_t getId() const;
  std::string getName() const;

protected:
  UrScriptHeaders urScriptHeaders;
  DispatchMotionsAsyncCb dispatchMotionsAsyncCb;

private:
  std::string _name;
  int32_t _id { };
};

#endif /* UR_CONTROL_COMMON_MOTIONSEQUENCE_H_ */
