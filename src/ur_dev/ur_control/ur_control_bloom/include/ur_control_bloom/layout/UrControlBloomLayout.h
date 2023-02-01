#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUT_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUT_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/layout/UrControlCommonLayout.h"

//Own components headers
#include "manager_utils/drawing/Image.h"

//Forward declarations
class InputEvent;
class UrControlBloomLayoutInitHelper;
struct UrControlBloomLayoutConfig;

class UrControlBloomLayout {
public:
  friend class UrControlBloomLayoutInitHelper;

  ErrorCode init(const UrControlBloomLayoutConfig& cfg,
                 const UrControlCommonLayoutOutInterface& commonOutInterface,
                 UrControlCommonLayoutInterface& commonInterface);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent& e);
  void process();

private:
  UrControlCommonLayout _commonLayout;
  Image _rose;
  Image _jenga;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUT_H_ */
