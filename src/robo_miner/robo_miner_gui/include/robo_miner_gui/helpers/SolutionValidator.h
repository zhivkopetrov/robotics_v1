#ifndef ROBO_MINER_GUI_SOLUTIONVALIDATOR_H_
#define ROBO_MINER_GUI_SOLUTIONVALIDATOR_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <string>
#include <vector>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers
#include "robo_miner_gui/defines/RoboMinerGuiDefines.h"

//Forward declarations
struct SolutionValidatorConfig;

class SolutionValidator {
public:
  int32_t init(const SolutionValidatorConfig &cfg,
               const GetFieldDescriptionCb &getFieldDescriptionCb);

  bool validateFieldMap(const std::vector<uint8_t> &rawData, uint32_t rows,
                        uint32_t cols, std::string &outError) const;

  //sequence will be sorted
  bool validateLongestSequence(CrystalSequence& sequence) const;

private:
  CrystalSequence _longestSequence;
  GetFieldDescriptionCb _getFieldDescriptionCb;
};

#endif /* ROBO_MINER_GUI_SOLUTIONVALIDATOR_H_ */
