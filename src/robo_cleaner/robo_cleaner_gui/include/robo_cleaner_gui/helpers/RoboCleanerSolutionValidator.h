#ifndef ROBO_CLEANER_GUI_SOLUTIONVALIDATOR_H_
#define ROBO_CLEANER_GUI_SOLUTIONVALIDATOR_H_

//System headers
#include <cstdint>
#include <string>
#include <vector>
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

  ValidationResult validateFieldMap(const std::vector<uint8_t> &rawData,
                                    uint32_t rows, uint32_t cols,
                                    std::string &outError);

  char getApproachingTileMarker(MoveType moveType) const;

  MoveValidation finishMove(const RobotState &state, MoveOutcome outcome,
                            MoveType moveType);

  void increaseTotalRobotMovesCounter(int32_t movesCount);
  int32_t getTotalRobotMovesCounter() const;

private:
  ErrorCode initOutInterface(
      const RoboCleanerSolutionValidatorOutInterface &outInterface);

  struct ValidationOptions {
    bool initialRobotStateRequested = false;
    bool fieldMapReveleaded = false;
    bool fieldMapValidated = false;
    bool fieldMapCleaned = false;
    std::vector<bool> longestSequenceValidationPoints;

    int32_t fieldMapValidationsTriesLeft = 3;
  };

  RoboCleanerSolutionValidatorOutInterface _outInterface;
  std::set<FieldPos> _reveleadMapTiles;
  ValidationOptions _validationOptions;

  int32_t _totalRobotMoves { };
};

#endif /* ROBO_CLEANER_GUI_SOLUTIONVALIDATOR_H_ */
