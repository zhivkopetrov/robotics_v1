//Corresponding header
#include "robo_collector_gui/field/Field.h"

//C system headers

//C++ system headers
#include <sstream>

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/config/FieldConfig.h"

int32_t Field::init(const FieldConfig &cfg) {
  if (0 >= cfg.rows || 0 >= cfg.cols) {
    LOGERR("Invalid configuration, rows: %d, cols: %d. Both 'rows' and 'cols' "
        "needs to be positive number", cfg.rows, cfg.cols);
    return FAILURE;
  }

  _emptyDataMarker = cfg.emptyTileMarker;

  TileConfig tileCfg;
  tileCfg.debugFontRsrcId = cfg.debugFontRsrcId;

  _fieldData.resize(cfg.rows);
  _tiles.resize(cfg.rows);
  for (int32_t row = 0; row < cfg.rows; ++row) {
    _fieldData[row].resize(cfg.cols, cfg.emptyTileMarker);
    _tiles[row].resize(cfg.cols);
    tileCfg.row = row;
    for (int32_t col = 0; col < cfg.cols; ++col) {
      tileCfg.tileRsrcId = cfg.tileRsrcId;
      tileCfg.col = col;
      tileCfg.screenCoordinates.x = col * cfg.tileWidth;
      tileCfg.screenCoordinates.y = row * cfg.tileHeight;

      //apply field offset
      tileCfg.screenCoordinates.x += cfg.fieldDimensions.x;
      tileCfg.screenCoordinates.y += cfg.fieldDimensions.y;
      if (SUCCESS != _tiles[row][col].init(tileCfg)) {
        LOGERR("_tiles[%d][%d].init() failed", row, col);
        return FAILURE;
      }
    }
  }

  _fieldFbo.create(cfg.fieldDimensions);
  _fieldFbo.activateAlphaModulation();
  _fieldFbo.setResetColor(Colors::FULL_TRANSPARENT);
  updateFieldFbo();

  return SUCCESS;
}

void Field::draw() const {
  _fieldFbo.draw();
}

void Field::updateFieldFbo() {
  _fieldFbo.unlock();
  _fieldFbo.reset();

  for (const auto & row : _tiles) {
    for (const auto &tile : row) {
      tile.drawOnFbo(_fieldFbo);
    }
  }

  _fieldFbo.update();
  _fieldFbo.lock();
}

void Field::setFieldDataMarker(const FieldPos &fieldPos, char fieldMarker) {
  _fieldData[fieldPos.row][fieldPos.col] = fieldMarker;
//  printFieldData();
}

void Field::resetFieldDataMarker(const FieldPos &fieldPos) {
//  LOGC("Resetting FieldData for row, col [%d,%d]", fieldPos.row, fieldPos.col);
  setFieldDataMarker(fieldPos, _emptyDataMarker);
}

const FieldData& Field::getFieldData() const {
  return _fieldData;
}

void Field::toggleDebugTexts() {
  for (auto& row : _tiles) {
    for (auto& tile : row) {
      tile.toggleDebugText();
    }
  }
  updateFieldFbo();
}

void Field::printFieldData() const {
  std::ostringstream ostr;
  for (const auto& row : _fieldData) {
    for (const auto field : row) {
      ostr << field;
    }
    ostr << '\n';
  }
  ostr << '\n';
  LOGC("Printing FieldData:\n%s", ostr.str().c_str());
}

