//Corresponding header
#include "robo_common/layout/field/Tile.h"

//System headers
#include <string>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"

//Own components headers
#include "robo_common/layout/field/config/TileConfig.h"

namespace {
constexpr auto DEBUG_TEXT_OFFSET_X = 10;
constexpr auto DEBUG_TEXT_OFFSET_Y = 10;
}

ErrorCode Tile::init(const TileConfig &cfg) {
  _tileImg.create(cfg.tileRsrcId);
  _tileImg.setPosition(cfg.screenCoordinates);

  std::string debugText;
  debugText.reserve(8); //max debug string size
  debugText.append("[").append(std::to_string(cfg.row)).append(",").append(
      std::to_string(cfg.col)).append("]");
  _debugText.create(cfg.debugFontRsrcId, debugText.c_str(), Colors::WHITE,
      Point(cfg.screenCoordinates.x + DEBUG_TEXT_OFFSET_X,
            cfg.screenCoordinates.y + DEBUG_TEXT_OFFSET_Y));
  _debugText.hide();

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

