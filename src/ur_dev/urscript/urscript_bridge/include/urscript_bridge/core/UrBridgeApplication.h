#ifndef URSCRIPT_BRIDGE_URBRIDGEAPPLICATION_H_
#define URSCRIPT_BRIDGE_URBRIDGEAPPLICATION_H_

//System headers
#include <vector>
#include <memory>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2Communicator.h"
#include "game_engine/defines/DependencyDefines.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "urscript_bridge/config/UrBridgeConfig.h"
#include "urscript_bridge/external_api/UrBridgeExternalInterface.h"

//Forward declarations

class UrBridgeApplication : public NonCopyable, public NonMoveable {
public:
  ~UrBridgeApplication() noexcept;

  ErrorCode loadDependencies(
      const std::vector<DependencyDescription> &dependencies);

  ErrorCode init(const UrBridgeConfig &cfg);

  ErrorCode run();

private:
  void deinit();
  void unloadDependencies();

  std::vector<DependencyDescription> _dependencies;

  std::unique_ptr<Ros2Communicator> _communicator;
  std::shared_ptr<UrBridgeExternalInterface> _externalInterface;
};

#endif /* URSCRIPT_BRIDGE_URBRIDGEAPPLICATION_H_ */
