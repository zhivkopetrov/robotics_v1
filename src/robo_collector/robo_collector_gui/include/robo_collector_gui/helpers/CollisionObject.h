#ifndef ROBO_COLLECTOR_GUI_COLLISIONOBJECT_H_
#define ROBO_COLLECTOR_GUI_COLLISIONOBJECT_H_

//C system headers

//C++ system headers
#include <cstddef>

//Other libraries headers
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

using CollisionObjHandle = size_t;

enum class CollisionWatchStatus {
  ON, OFF
};

class CollisionObject {
public:
  virtual ~CollisionObject() = default;

  virtual void registerCollision(const Rectangle& intersectRect) = 0;
  virtual Rectangle getBoundary() const = 0;

protected:
  CollisionObjHandle _collisionObjHandle{};
};

#endif /* ROBO_COLLECTOR_GUI_COLLISIONOBJECT_H_ */
