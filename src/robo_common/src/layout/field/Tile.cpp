//Corresponding header
#include "robo_common/layout/field/Tile.h"

//System headers
#include <string>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"

//Own components headers
#include "robo_common/layout/field/config/TileConfig.h"

ErrorCode Tile::init(const TileConfig &cfg) {
  _tileImg.create(cfg.tileRsrcId);
  _tileImg.activateScaling();
  _tileImg.setScaledWidth(cfg.width);
  _tileImg.setScaledHeight(cfg.height);
  _tileImg.setPosition(cfg.screenCoordinates);

  createDebugText(cfg);

  return ErrorCode::SUCCESS;
}

void Tile::draw() const {
  _tileImg.draw();
  _debugText.draw();
}

void Tile::drawOnFbo(Fbo &fbo) const {
  fbo.addWidget(_tileImg);

  if (_debugText.isVisible()) {
    fbo.addWidget(_debugText);
  }
}

void Tile::toggleDebugText() {
  _debugText.isVisible() ? _debugText.hide() : _debugText.show();
}

void Tile::createDebugText(const TileConfig &cfg) {
  constexpr double textToTileRatio = 0.25;
  const int32_t offsetFromTileX = static_cast<int32_t>(0.05 * cfg.width);
  const int32_t offsetFromTileY = static_cast<int32_t>(0.05 * cfg.height);

  const int32_t maxScaledWidth = textToTileRatio * cfg.width;
  const int32_t maxScaledHeight = textToTileRatio * cfg.height;
  const Point offset = Point(offsetFromTileX, offsetFromTileY);
  Point absPos = cfg.screenCoordinates;
  absPos += offset;

  std::string debugText;
  debugText.reserve(8); //max debug string size
  debugText.append("[").append(std::to_string(cfg.row)).append(",").append(
      std::to_string(cfg.col)).append("]");
  _debugText.create(cfg.debugFontRsrcId, debugText.c_str(), Colors::CYAN,
      absPos);
  _debugText.activateScaling();
  _debugText.setMaxScalingWidth(maxScaledWidth);
  _debugText.setMaxScalingHeight(maxScaledHeight);
  _debugText.hide();
}

