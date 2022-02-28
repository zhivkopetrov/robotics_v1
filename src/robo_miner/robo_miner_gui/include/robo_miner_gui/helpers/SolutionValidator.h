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

//Forward declarations

class SolutionValidator {
public:
  int32_t init(const GetFieldDescriptionCb &getFieldDescriptionCb);

  bool validateSolution(const std::vector<uint8_t> &rawData, uint32_t rows,
                        uint32_t cols, std::string &outError) const;

private:
  GetFieldDescriptionCb _getFieldDescriptionCb;
};

#endif /* ROBO_MINER_GUI_SOLUTIONVALIDATOR_H_ */
