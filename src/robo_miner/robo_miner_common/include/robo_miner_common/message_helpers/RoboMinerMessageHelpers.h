#ifndef ROBO_MINER_COMMON_ROBOMINERMESSAGEHELPERS_H_
#define ROBO_MINER_COMMON_ROBOMINERMESSAGEHELPERS_H_

//System headers

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"

//Own components headers

//Forward declarations

int8_t getMoveTypeField(MoveType moveType);
MoveType getMoveType(int8_t moveType);

int32_t getRobotDirectionField(Direction dir);
Direction getRobotDirection(int32_t dir);

#endif /* ROBO_MINER_COMMON_ROBOMINERMESSAGEHELPERS_H_ */
