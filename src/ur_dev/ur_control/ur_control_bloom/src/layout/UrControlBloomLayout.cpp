//Corresponding header
#include "ur_control_bloom/layout/UrControlBloomLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/layout/helpers/UrControlBloomLayoutInitHelper.h"

ErrorCode UrControlBloomLayout::init(
  const UrControlBloomLayoutConfig& cfg,
  const UrControlCommonLayoutOutInterface& commonOutInterface,
  UrControlCommonLayoutInterface &commonInterface) {
  if (ErrorCode::SUCCESS != UrControlBloomLayoutInitHelper::init(
        cfg, commonOutInterface, commonInterface, *this)) {
    LOGERR("Error, UrControlBloomLayoutInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void UrControlBloomLayout::deinit() {
  _commonLayout.deinit();
}

void UrControlBloomLayout::draw() const {
  _commonLayout.draw();
}

void UrControlBloomLayout::handleEvent(const InputEvent &e) {
  _commonLayout.handleEvent(e);
}

void UrControlBloomLayout::process() {

}

