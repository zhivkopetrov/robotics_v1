//Corresponding header
#include "robo_collector_gui/robo_miner/field/RoboMinerField.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/robo_miner/field/config/RoboMinerFieldConfig.h"

namespace {
//TODO remove this temporary hack
int32_t generateMapConfig(FieldData& data) {
  const auto rows = data.size();
  if (rows != Defines::FIELD_ROWS) {
    const auto cols = data[0].size();
    if (cols != Defines::FIELD_COLS) {
      LOGERR("Only %zux%zu is matrix is supported at the moment", rows, cols);
      return FAILURE;
    }
  }

  data = {
      {'r', 'r', '.', '.', '.', 'b', 'r'},
      {'g', 'r', '.', 'c', 'c', 'r', 'r'},
      {'g', 'r', 'r', 'r', 'r', 'r', 'g'},
      {'g', 'r', 'c', 'c', 'c', 'g', 'g'},
      {'.', 'r', 'c', 'b', 'b', '.', 'g'},
      {'.', '.', '.', 'p', 'p', '.', '.'},
  };
  return SUCCESS;
}
}

int32_t RoboMinerField::init(const RoboMinerFieldConfig &cfg) {
  if (0 >= cfg.rows || 0 >= cfg.cols) {
    LOGERR("Invalid configuration, rows: %d, cols: %d. Both 'rows' and 'cols' "
        "needs to be positive number", cfg.rows, cfg.cols);
    return FAILURE;
  }

  _emptyDataMarker = cfg.emptyTileMarker;
  _fieldData.resize(cfg.rows);
  for (int32_t row = 0; row < cfg.rows; ++row) {
    _fieldData[row].resize(cfg.cols, cfg.emptyTileMarker);
  }

  if (SUCCESS != generateMapConfig(_fieldData)) {
    LOGERR("Error, generateMapConfig failed");
    return FAILURE;
  }

  return SUCCESS;
}

const FieldData& RoboMinerField::getFieldData() const {
  return _fieldData;
}

char RoboMinerField::getEmptyMarker() const {
  return _emptyDataMarker;
}

