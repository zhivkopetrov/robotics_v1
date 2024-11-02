//Corresponding header
#include "robo_common/layout/field/fog_of_war/FogOfWar.h"

//System headers

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "utils/log/Log.h"

//Own components headers
#include "robo_common/layout/field/config/FieldConfig.h"

ErrorCode FogOfWar::init(const FogOfWarConfig &fogCfg,
                         const FogOfWarOutInterface &outInterface,
                         const FieldConfig &fieldCfg) {
  _status = fogCfg.status;
  if (FogOfWarStatus::DISABLED == _status) {
    return ErrorCode::SUCCESS;
  }

  _outInterface = outInterface;
  if (nullptr == _outInterface.collisionWatcher) {
    LOGERR("Error, nullptr provided for collisionWatcher");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != createFogTiles(fieldCfg,
      fogCfg.playerStartingPos, fogCfg.fogTilesFadeAnimTimerIds)) {
    LOGERR("createFogTiles() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != populateFogTiles(fogCfg.cloudRsrcId)) {
    LOGERR("createFogTiles() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void FogOfWar::draw() const {
  if (FogOfWarStatus::DISABLED == _status) {
    return;
  }

  for (const auto &[_, fogTile] : _fogTiles) {
    fogTile.draw();
  }
}

void FogOfWar::revealAllFogTiles() {
  if (_isRevealAllFogTilesRequested) {
    return;
  }
  _isRevealAllFogTilesRequested = true;

  for (auto &[_, fogTile] : _fogTiles) {
    fogTile.startAnimation();
  }
}

ErrorCode FogOfWar::createFogTiles(
    const FieldConfig &fieldCfg,
    const FieldPos &playerStartingPos,
    const std::vector<int> &fogTilesFadeAnimTimerIds) {
  const auto &fieldDescr = fieldCfg.description;
  Rectangle cloudFboDimensions { 0, 0, fieldDescr.tileWidth,
      fieldDescr.tileHeight };

  const int32_t tilesCount = fieldDescr.rows * fieldDescr.cols;
  if (tilesCount != static_cast<int32_t>(fogTilesFadeAnimTimerIds.size())) {
    LOGERR("Error, tiles count mismatch provided fog timer anim timer ids");
    return ErrorCode::FAILURE;
  }

  const auto onFogObjectAimCompleteCb = std::bind(
      &FogOfWar::onFogObjectAnimComplete, this, std::placeholders::_1);

  _fogTiles.reserve(tilesCount - 1);
  int32_t currTileId = 0;
  FieldPos currrFieldPos;
  for (int32_t row = 0; row < fieldDescr.rows; ++row) {
    currrFieldPos.row = row;
    cloudFboDimensions.y = RoboCommonDefines::FIRST_TILE_Y_POS
        + (row * fieldDescr.tileHeight);

    for (int32_t col = 0; col < fieldDescr.cols; ++col) {
      currrFieldPos.col = col;
      cloudFboDimensions.x = RoboCommonDefines::FIRST_TILE_X_POS
          + (col * fieldDescr.tileWidth);

      if (currrFieldPos == playerStartingPos) {
        continue; //skip the starting position of the robot
      }

      if (ErrorCode::SUCCESS != _fogTiles[currTileId].init(cloudFboDimensions,
              currTileId, fogTilesFadeAnimTimerIds[currTileId],
              onFogObjectAimCompleteCb, _outInterface.collisionWatcher)) {
        LOGERR("_fogTiles[%d].init() failed", currTileId);
        return ErrorCode::FAILURE;
      }

      ++currTileId;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode FogOfWar::populateFogTiles(uint64_t cloudRsrcId) {
  Image fogImg;
  fogImg.create(cloudRsrcId);
  fogImg.setPosition(RoboCommonDefines::FIRST_TILE_X_POS,
      RoboCommonDefines::FIRST_TILE_Y_POS);

  for (auto &[_, fogTile] : _fogTiles) {
    fogTile.populateVisualContent(fogImg);
  }

  return ErrorCode::SUCCESS;
}

void FogOfWar::onFogObjectAnimComplete(int32_t id) {
  auto it = _fogTiles.find(id);
  if (_fogTiles.end() == it) {
    LOGERR("Error, fog tile with Id: %d could not be found", id);
    return;
  }

  _fogTiles.erase(id);
}

