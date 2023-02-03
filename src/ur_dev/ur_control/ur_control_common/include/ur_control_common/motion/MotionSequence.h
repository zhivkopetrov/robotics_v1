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
#include "urscript_common/defines/UrScriptDefines.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

using UrScriptHeaders = std::unordered_map<std::string, UrScriptPayload>;
using MotionSequenceActionDoneCb = std::function<void()>;

class MotionSequence : public NonCopyable, public NonMoveable { 
public:
  MotionSequence(
    const std::string& name, int32_t id, UrScriptHeaders&& headers);
  virtual ~MotionSequence() noexcept = default;

  virtual ErrorCode init(const std::any& cfg) = 0;
  virtual void start(const MotionSequenceActionDoneCb& cb) = 0;
  virtual void gracefulStop(const MotionSequenceActionDoneCb& cb) = 0;
  virtual void abort(const MotionSequenceActionDoneCb& cb) = 0;
  virtual void recover(const MotionSequenceActionDoneCb& cb) = 0;

  int32_t getId() const;
  std::string getName() const;

protected:
  std::queue<UrScriptPayload> loadedMotionCommands;
  const UrScriptHeaders urScriptHeaders;

private:
  std::string _name;
  int32_t _id { };
};

#endif /* UR_CONTROL_COMMON_MOTIONSEQUENCE_H_ */
