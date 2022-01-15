//Corresponding header
#include "robo_collector_gui/field/Field.h"

//C system headers

//C++ system headers

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

  TileConfig tileCfg;
  tileCfg.debugFontRsrcId = cfg.debugFontRsrcId;

  _tiles.resize(cfg.rows);
  for (int32_t row = 0; row < cfg.rows; ++row) {
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

  _fieldSB.create(cfg.fieldDimensions);
  updateFieldSpriteBuffer();

  return SUCCESS;
}

void Field::handleEvent([[maybe_unused]]const InputEvent &e) {

}

void Field::draw() const {
  _fieldSB.draw();
}

void Field::updateFieldSpriteBuffer() {
  _fieldSB.unlock();
  _fieldSB.reset();
  for (const auto & row : _tiles) {
    for (const auto &tile : row) {
      tile.drawOnSpriteBuffer(_fieldSB);
    }
  }
  _fieldSB.update();
  _fieldSB.lock();
}

