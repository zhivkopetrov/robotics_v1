#ifndef UR_CONTROL_COMMON_MOTIONSEQUENCE_H_
#define UR_CONTROL_COMMON_MOTIONSEQUENCE_H_

//System headers
#include <cstdint>
#include <string>
#include <queue>
#include <functional>

//Other libraries headers
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct MotionCommand {
  std::string data;
};

using MotionSequenceActionDoneCb = std::function<void()>;

class MotionSequence : public NonCopyable, public NonMoveable { 
public:
  MotionSequence(
    const std::string& name, int32_t id, 
    std::queue<MotionCommand> &&motionCommands);
  virtual ~MotionSequence() = default;

  virtual void start(const MotionSequenceActionDoneCb& cb) = 0;
  virtual void gracefulStop(const MotionSequenceActionDoneCb& cb) = 0;
  virtual void abort(const MotionSequenceActionDoneCb& cb) = 0;
  virtual void recover(const MotionSequenceActionDoneCb& cb) = 0;

  int32_t getId() const;
  std::string getName() const;

protected:
  std::queue<MotionCommand> _motionCommands;
  MotionSequenceActionDoneCb _actionDoneCb;

private:
  std::string _name;
  int32_t _id { };
};

#endif /* UR_CONTROL_COMMON_MOTIONSEQUENCE_H_ */
