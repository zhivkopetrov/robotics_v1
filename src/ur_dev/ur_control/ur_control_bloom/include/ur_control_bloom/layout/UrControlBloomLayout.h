#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUT_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUT_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "ur_control_common/layout/UrControlCommonLayout.h"

//Own components headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"

//Forward declarations
class InputEvent;
struct UrControlBloomLayoutConfig;

class UrControlBloomLayout : public UrControlCommonLayout {
public:
  ErrorCode init(const UrControlBloomLayoutConfig& cfg,
                 const UrControlCommonLayoutOutInterface& commonOutInterface,
                 UrControlCommonLayoutInterface& commonInterface);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent& e);
  void process();

  void enterInitState();
  void exitInitState();
  void enterIdleState();
  void exitIdleState();
  void enterBloomState();
  void exitBloomState();
  void enterBloomRecoveryState();
  void exitBloomRecoveryState();
  void enterJengaState();
  void exitJengaState();
  void enterJengaRecoveryState();
  void exitJengaRecoveryState();

private:
  ErrorCode initStandaloneEntities(const UrControlBloomLayoutConfig &cfg);

  Image _rose;
  Image _jenga;
  Text _stateTextHeader;
  Text _stateText;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUT_H_ */
