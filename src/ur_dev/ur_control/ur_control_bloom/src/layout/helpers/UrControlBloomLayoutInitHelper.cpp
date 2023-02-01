//Corresponding header
#include "ur_control_bloom/layout/helpers/UrControlBloomLayoutInitHelper.h"

//System headers
#include <algorithm>
#include <numeric>

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/layout/config/UrControlBloomLayoutConfig.h"
#include "ur_control_bloom/layout/UrControlBloomLayout.h"

using namespace std::placeholders;

ErrorCode UrControlBloomLayoutInitHelper::init(
  const UrControlBloomLayoutConfig &cfg,
  const UrControlCommonLayoutOutInterface& commonOutInterface,
  UrControlCommonLayoutInterface &commonInterface,
  UrControlBloomLayout &layout) {

  if (ErrorCode::SUCCESS != layout._commonLayout.init(cfg.commonLayoutCfg,
          commonOutInterface, commonInterface)) {
    LOGERR("_commonLayout.init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initStandaloneImages(cfg, layout)) {
    LOGERR("initStandaloneImages() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomLayoutInitHelper::initStandaloneImages(
  [[maybe_unused]]const UrControlBloomLayoutConfig &cfg,
  [[maybe_unused]]UrControlBloomLayout &layout) {


  return ErrorCode::SUCCESS;                                    
}

