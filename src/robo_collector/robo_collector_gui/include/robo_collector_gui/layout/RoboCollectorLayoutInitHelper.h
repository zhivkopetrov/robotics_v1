#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/entities/robot/RobotAI.h"

//Forward declarations
class CollisionWatcher;
class RoboCollectorLayout;
struct RoboCollectorLayoutConfig;
struct PanelHandlerConfig;
struct CoinHandlerConfig;
struct RoboCollectorControllerConfig;

struct RoboCollectorLayoutInterface {
  EnablePlayerInputCb enablePlayerInputCb;
  GetFieldDataCb getFieldDataCb;
  MoveButtonClickCb moveButtonClickCb;
  std::vector<RobotActInterface> robotActInterfaces;
};

struct RoboCollectorLayoutOutInterface {
  IsPlayerTurnActiveCb isPlayerTurnActiveCb;
  FinishRobotActCb finishRobotActCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

class RoboCollectorLayoutInitHelper {
public:
  int32_t init(const RoboCollectorLayoutConfig& cfg,
               const RoboCollectorLayoutOutInterface& interface,
               RoboCollectorLayout& layout);

private:
  int32_t initRobots(const RoboCollectorLayoutConfig& layoutCfg,
                     const RoboCollectorLayoutOutInterface& interface,
                     RoboCollectorLayout& layout);

  int32_t initPanelHandler(const PanelHandlerConfig& cfg,
                           RoboCollectorLayout& layout);

  int32_t initCoinHandler(const CoinHandlerConfig& cfg,
                          const RoboCollectorLayoutOutInterface& interface,
                          RoboCollectorLayout& layout);

  int32_t initController(const RoboCollectorControllerConfig& guiCfg,
                         RoboCollectorLayout& layout);
};

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORLAYOUTINITHELPER_H_ */
