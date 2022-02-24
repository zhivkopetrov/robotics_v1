//Corresponding header
#include "robo_common/layout/field/Field.h"

//C system headers

//C++ system headers
#include <sstream>
#include <algorithm>
#include <numeric>

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/field/config/FieldConfig.h"
#include "robo_common/layout/field/config/TileConfig.h"

int32_t Field::init(const FieldConfig &cfg) {
  if (0 >= cfg.description.rows || 0 >= cfg.description.cols) {
    LOGERR("Invalid configuration, rows: %d, cols: %d. Both 'rows' and 'cols' "
           "needs to be positive number", cfg.description.rows,
           cfg.description.cols);
    return FAILURE;
  }

  if (SUCCESS != initTiles(cfg)) {
    LOGERR("Error, initTiles() failed");
    return FAILURE;
  }

  _description = cfg.description;

  const auto fieldWidth = cfg.description.cols * cfg.description.tileWidth;
  const auto fieldHeight = cfg.description.rows * cfg.description.tileHeight;
  const auto fieldDimensios = Rectangle(RoboCommonDefines::FIRST_TILE_X_POS,
      RoboCommonDefines::FIRST_TILE_Y_POS, fieldWidth, fieldHeight);

  _fieldFbo.create(fieldDimensios);
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

  for (const auto &tile : _tiles) {
    tile.drawOnFbo(_fieldFbo);
  }

  _fieldFbo.update();
  _fieldFbo.lock();
}

void Field::setFieldDataMarker(const FieldPos &fieldPos, char fieldMarker) {
  _description.data[fieldPos.row][fieldPos.col] = fieldMarker;
//  printFieldData();
}

void Field::resetFieldDataMarker(const FieldPos &fieldPos) {
//  LOGC("Resetting FieldData for row, col [%d,%d]", fieldPos.row, fieldPos.col);
  setFieldDataMarker(fieldPos, _description.emptyDataMarker);
}

const FieldData& Field::getFieldData() const {
  return _description.data;
}

const FieldDescription& Field::getDescription() const {
  return _description;
}

void Field::toggleDebugTexts() {
  for (auto &tile : _tiles) {
    tile.toggleDebugText();
  }
  updateFieldFbo();
}

int32_t Field::initTiles(const FieldConfig &cfg) {
  TileConfig tileCfg;
  tileCfg.tileRsrcId = cfg.tileRsrcId;
  tileCfg.debugFontRsrcId = cfg.debugFontRsrcId;

  const auto hardObstacleMarker = cfg.description.hardObstacleMarker;
  const auto tilesCount = std::accumulate(cfg.description.data.begin(),
      cfg.description.data.end(), 0,
      [hardObstacleMarker](auto count, const auto &row) {
        return count
            + std::count_if(std::begin(row), std::end(row),
                [hardObstacleMarker](auto marker) {
                  return marker != hardObstacleMarker;
                });
      });

  _tiles.resize(tilesCount);
  int32_t currTileId = 0;

  for (int32_t row = 0; row < cfg.description.rows; ++row) {
    tileCfg.screenCoordinates.y = RoboCommonDefines::FIRST_TILE_Y_POS +
        (row * cfg.description.tileHeight);

    tileCfg.row = row;
    for (int32_t col = 0; col < cfg.description.cols; ++col) {
      const auto currMarker = cfg.description.data[row][col];
      if (cfg.description.hardObstacleMarker == currMarker) {
        continue;
      }

      tileCfg.col = col;
      tileCfg.screenCoordinates.x = RoboCommonDefines::FIRST_TILE_X_POS +
          (col * cfg.description.tileWidth);

      if (SUCCESS != _tiles[currTileId].init(tileCfg)) {
        LOGERR("_tiles[%d].init() failed for row, col: [%d,%d]",
            currTileId, row, col);
        return FAILURE;
      }
      ++currTileId;
    }
  }

  return SUCCESS;
}

void Field::printFieldData() const {
  std::ostringstream ostr;
  for (const auto &row : _description.data) {
    for (const auto field : row) {
      ostr << field;
    }
    ostr << '\n';
  }
  ostr << '\n';
  LOGC("Printing FieldData:\n%s", ostr.str().c_str());
}

