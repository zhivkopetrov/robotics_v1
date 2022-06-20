#ifndef ROBO_COMMON_ROBOCOMMONLAYOUT_H_
#define ROBO_COMMON_ROBOCOMMONLAYOUT_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/layout/helpers/RoboCommonLayoutInterfaces.h"
#include "robo_common/layout/field/Field.h"
#include "robo_common/layout/field/fog_of_war/FogOfWar.h"
#include "robo_common/layout/entities/robot/Robot.h"
#include "robo_common/layout/animators/GameEndAnimator.h"
#include "robo_common/layout/animators/AchievementAnimator.h"

//Forward declarations
class RoboCommonLayoutInitHelper;
struct RoboCommonLayoutConfig;

class RoboCommonLayout {
  friend class RoboCommonLayoutInitHelper;

public:
  ErrorCode init(const RoboCommonLayoutConfig& cfg,
                 const RoboCommonLayoutOutInterface &outInterface,
                 RoboCommonLayoutInterface &interface);
  void deinit();
  void draw() const;

  //contains FogOfWar
  void drawSecondLayer() const;

  //contains FogOfWar
  void drawThirdLayer() const;

  //TODO create a separate help page class
  void toggleHelpPage();

  void toggleDebugInfo();

private:
  RoboCommonLayoutInterface produceInterface();

  Image _map;
  Field _field;
  FogOfWar _fogOfWar;
  Robot _playerRobot;
  GameEndAnimator _gameEndAnimator;
  AchievementAnimator _achievementAnimator;
};

#endif /* ROBO_COMMON_ROBOCOMMONLAYOUT_H_ */
