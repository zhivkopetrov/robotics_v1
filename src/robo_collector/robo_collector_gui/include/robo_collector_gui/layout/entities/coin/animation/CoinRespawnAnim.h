#ifndef ROBO_COLLECTOR_COINRESPAWNANIM_H_
#define ROBO_COLLECTOR_COINRESPAWNANIM_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <functional>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/time/TimerClient.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"

//Forward declarations

struct CoinRespawnAnimConfig {
  std::function<void(CoinAnimType)> animEndCb;
  Image* coinImg = nullptr;
  int32_t timerId = 0;
};

class CoinRespawnAnim : public TimerClient {
public:
  int32_t init(const CoinRespawnAnimConfig& cfg);

  void start();

private:
  void onTimeout(const int32_t timerId) override;

  void processAnim();

  std::function<void(CoinAnimType)> _animEndCb;
  Image* _coinImg = nullptr;
  int32_t _timerId = 0;
  int32_t animationSteps = 0;
};

#endif /* ROBO_COLLECTOR_COINRESPAWNANIM_H_ */
