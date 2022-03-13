#ifndef ROBO_MINER_GUI_SOLUTIONVALIDATOR_H_
#define ROBO_MINER_GUI_SOLUTIONVALIDATOR_H_

//System headers
#include <cstdint>
#include <string>
#include <vector>
#include <set>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_miner_gui/defines/RoboMinerGuiDefines.h"

//Forward declarations
struct SolutionValidatorConfig;

struct SolutionValidatorOutInterface {
  GetFieldDescriptionCb getFieldDescriptionCb;
  GetRobotStateCb getRobotStateCb;
};

struct ValidationResult {
  bool success = true;
  bool majorError = false;
};

class SolutionValidator {
public:
  ErrorCode init(const SolutionValidatorConfig &cfg,
                 const SolutionValidatorOutInterface &outInterface);

  void fieldMapRevealed();

  ValidationResult validateFieldMap(const std::vector<uint8_t> &rawData,
                                    uint32_t rows, uint32_t cols,
                                    std::string &outError);

  //sequence will be sorted
  ValidationResult validateLongestSequence(CrystalSequence &sequence,
                                           std::string &outError);

  ValidationResult handleNormalMove(const FieldPos &fieldPos);
  ValidationResult handleMiningMove(const FieldPos &fieldPos);

  ValidationResult validateActivateMining(std::string &outError);

  bool isMiningActive() const;

private:
  ErrorCode initOutInterface(const SolutionValidatorOutInterface &outInterface);

  bool validateMiningPos(const FieldPos &fieldPos);

  struct ValidationOptions {
    bool fieldMapReveleaded = false;
    bool fieldMapValidated = false;
    bool longestSequenceValidated = false;
    bool miningActivated = false;
    std::vector<bool> longestSequenceValidationPoints;

    int32_t fieldMapValidationsTriesLeft = 3;
    int32_t longestSequenceValidationsTriesLeft = 3;

    size_t targetMapTilesCount = 0;
  };

  SolutionValidatorOutInterface _outInterface;
  CrystalSequence _longestSequence;
  std::set<FieldPos> _reveleadMapTiles;
  ValidationOptions _validationOptions;
};

#endif /* ROBO_MINER_GUI_SOLUTIONVALIDATOR_H_ */
