#ifndef ROBO_COMMON_TILE_H_
#define ROBO_COMMON_TILE_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"

//Own components headers

//Forward declarations
class Fbo;
struct TileConfig;
struct TileDebugConfig;

class Tile {
public:
  int32_t init(const TileConfig &cfg);

  void draw() const;

  void drawOnFbo(Fbo &fbo) const;

  void toggleDebugText();

private:
  Image _tileImg;
  Text _debugText;
};

#endif /* ROBO_COMMON_TILE_H_ */
