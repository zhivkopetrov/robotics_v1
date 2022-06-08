#ifndef ROBO_CLEANER_GUI_SOLUTIONVALIDATOR_H_
#define ROBO_CLEANER_GUI_SOLUTIONVALIDATOR_H_

//System headers
#include <cstdint>
#include <string>
#include <set>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiDefines.h"

//Forward declarations
struct RoboCleanerSolutionValidatorConfig;

struct RoboCleanerSolutionValidatorOutInterface {
  GetFieldDescriptionCb getFieldDescriptionCb;
  GetRobotStateCb getRobotStateCb;
};

struct ValidationResult {
  bool success = true;
  bool majorError = false;
};

struct MoveValidation {
  bool tileRevealed = false;
  bool tileCleaned = false;
  char processedMarker = RoboCommonDefines::UNKNOWN_FIELD_MARKER;
  bool reachedEndGameCondition = false;
};

struct InitialRobotState {
  Direction robotDir = Direction::UP;
  uint8_t robotTile = RoboCommonDefines::UNKNOWN_FIELD_MARKER;
};

class RoboCleanerSolutionValidator {
public:
  ErrorCode init(const RoboCleanerSolutionValidatorConfig &cfg,
                 const RoboCleanerSolutionValidatorOutInterface &outInterface);

  ValidationResult queryInitialRobotPos(InitialRobotState &outRobotState,
                                        std::string &outError);

  void fieldMapRevealed();

  void fieldMapCleaned();

  char getApproachingTileMarker(MoveType moveType) const;

  MoveValidation finishMove(const RobotState &state, MoveOutcome outcome,
                            MoveType moveType);

  void increaseTotalRobotMovesCounter(int32_t movesCount);
  int32_t getTotalRobotMovesCounter() const;

  bool isRobotAtChargingStation() const;

private:
  ErrorCode initOutInterface(
      const RoboCleanerSolutionValidatorOutInterface &outInterface);

  struct ValidationOptions {
    bool initialRobotStateRequested = false;
    bool fieldMapReveleaded = false;
    bool fieldMapCleaned = false;
  };

  RoboCleanerSolutionValidatorOutInterface _outInterface;
  std::set<FieldPos> _reveleadMapTiles;
  ValidationOptions _validationOptions;

  int32_t _totalRobotMoves { };
};

#endif /* ROBO_CLEANER_GUI_SOLUTIONVALIDATOR_H_ */
