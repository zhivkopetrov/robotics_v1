//Corresponding header
#include "robo_common/layout/field/Field.h"

//System headers
#include <sstream>
#include <algorithm>
#include <numeric>

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/field/config/FieldConfig.h"
#include "robo_common/layout/field/FieldUtils.h"
#include "robo_common/layout/field/config/TileConfig.h"

ErrorCode Field::init(const FieldConfig &cfg,
                      const FieldOutInterface &interface) {
  if (0 >= cfg.description.rows || 0 >= cfg.description.cols) {
    LOGERR("Invalid configuration, rows: %d, cols: %d. Both 'rows' and 'cols' "
           "needs to be positive number",
        cfg.description.rows, cfg.description.cols);
    return ErrorCode::FAILURE;
  }
  _description = cfg.description;

  if (ErrorCode::SUCCESS != initTiles(cfg, interface)) {
    LOGERR("Error, initTiles() failed");
    return ErrorCode::FAILURE;
  }

  const auto fieldWidth = cfg.description.cols * cfg.description.tileWidth;
  const auto fieldHeight = cfg.description.rows * cfg.description.tileHeight;
  const auto fieldDimensions = Rectangle(RoboCommonDefines::FIRST_TILE_X_POS,
      RoboCommonDefines::FIRST_TILE_Y_POS, fieldWidth, fieldHeight);

#if DEBUG_VISUAL_OVERLAY
  Rectangle visualOverlayDimensions = fieldDimensios;
  visualOverlayDimensions.x -= cfg.description.tileWidth;
  visualOverlayDimensions.w += (2 * cfg.description.tileWidth);
  visualOverlayDimensions.y -= cfg.description.tileHeight;
  visualOverlayDimensions.h += (2 * cfg.description.tileHeight);
  _fieldFbo.create(visualOverlayDimensions);
#else
  _fieldFbo.create(fieldDimensions);
#endif //DEBUG_VISUAL_OVERLAY
  _fieldFbo.activateAlphaModulation();
  _fieldFbo.setResetColor(Colors::FULL_TRANSPARENT);
  updateFieldFbo();

  return ErrorCode::SUCCESS;
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
  _obstacleHandler.drawOnFbo(_fieldFbo);

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

const FieldDescription& Field::getDescription() const {
  return _description;
}

void Field::toggleDebugTexts() {
  for (auto &tile : _tiles) {
    tile.toggleDebugText();
  }
  updateFieldFbo();
}

ErrorCode Field::initTiles(const FieldConfig &cfg,
                           const FieldOutInterface &interface) {
  TileConfig tileCfg;
  tileCfg.tileRsrcId = cfg.tileRsrcId;
  tileCfg.debugFontRsrcId = cfg.debugFontRsrcId;
  tileCfg.width = cfg.description.tileWidth;
  tileCfg.height = cfg.description.tileHeight;

  _tiles.resize(cfg.description.emptyTilesCount);
  int32_t currTileId = 0;
  FieldPos currFieldPos;
  std::vector<FieldPos> obstaclePositions;
  obstaclePositions.reserve(cfg.description.obstacleTilesCount);

  for (int32_t row = 0; row < cfg.description.rows; ++row) {
    tileCfg.screenCoordinates.y = RoboCommonDefines::FIRST_TILE_Y_POS
        + (row * cfg.description.tileHeight);

    currFieldPos.row = row;
    tileCfg.row = row;
    for (int32_t col = 0; col < cfg.description.cols; ++col) {
      currFieldPos.col = col;
      if (FieldUtils::collidesWithObstacle(currFieldPos, cfg.description)) {
        obstaclePositions.push_back(currFieldPos);
        continue;
      }

      tileCfg.col = col;
      tileCfg.screenCoordinates.x = RoboCommonDefines::FIRST_TILE_X_POS
          + (col * cfg.description.tileWidth);

      if (ErrorCode::SUCCESS != _tiles[currTileId].init(tileCfg)) {
        LOGERR("_tiles[%d].init() failed for row, col: [%d,%d]", currTileId,
            row, col);
        return ErrorCode::FAILURE;
      }
      ++currTileId;
    }
  }

  const int32_t obstaclesCount = static_cast<int32_t>(obstaclePositions.size());
  if (cfg.description.obstacleTilesCount != obstaclesCount) {
    LOGERR("Error, expected vs current obstacles mismatch [%d vs %d]",
        cfg.description.obstacleTilesCount, obstaclesCount);
    return ErrorCode::FAILURE;
  }

  const ObstacleHandlerOutInterface obsHandlerOutInterface = {
    .objechApproachOverlayTriggeredCb =
        interface.objechApproachOverlayTriggeredCb,
    .containerRedrawCb = std::bind(&Field::updateFieldFbo, this),
    .collisionWatcher = interface.collisionWatcher
  };

  if (ErrorCode::SUCCESS != _obstacleHandler.init(cfg.obstacleHandlerConfig,
          _description, obstaclePositions, obsHandlerOutInterface)) {
    LOGERR("Error, _obstacleHandler.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
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

