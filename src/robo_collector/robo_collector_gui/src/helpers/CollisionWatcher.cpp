//Corresponding header
#include "robo_collector_gui/helpers/CollisionWatcher.h"

//C system headers

//C++ system headers
#include <algorithm>

//Other libraries headers
#include "sdl_utils/drawing/GeometryUtils.h"
#include "utils/Log.h"

//Own components headers

CollisionObjHandle CollisionWatcher::registerObject(CollisionObject *object) {
  CollisionObjHandle handleIdx = 0;
  for (auto &obj : _objects) {
    // free index found, occupy it
    if (nullptr == obj) {
      obj = object;
      return handleIdx;
    }

    ++handleIdx;
  }

  _objects.push_back(object);
  return handleIdx;
}

void CollisionWatcher::unregisterObject(CollisionObjHandle handle) {
  if (handle >= _objects.size()) {
    LOGERR("CollisionObjHandle: %d not found", handle);
    return;
  }

  _objects[handle] = nullptr;
}

void CollisionWatcher::toggleWatchStatus(CollisionObjHandle handle,
                                         CollisionWatchStatus status) {
  if (handle >= _objects.size()) {
    LOGERR("CollisionObjHandle: %d not found", handle);
    return;
  }

  auto it = std::find(_activeWatchedHandles.begin(),
      _activeWatchedHandles.end(), handle);
  if (CollisionWatchStatus::ON == status) {
    if (it != _activeWatchedHandles.end()) {
      LOGERR("CollisionObjHandle: %d has already an active status", handle);
      return;
    }

    _activeWatchedHandles.push_back(handle);
  } else { // CollisionWatchStatus::OFF == status
    if (it == _activeWatchedHandles.end()) {
      LOGERR("CollisionObjHandle: %d is not active to be disabled", handle);
      return;
    }

    _activeWatchedHandles.erase(it);
  }
}

void CollisionWatcher::process() {
  const auto objSize = _objects.size();
  Rectangle intersectRect;
  for (const auto activeHandle : _activeWatchedHandles) {
    for (CollisionObjHandle handle = 0; handle < objSize; ++handle) {
      if (handle == activeHandle) {
        continue; //skip self-collision
      }

      const bool found = GeometryUtils::findRectIntersection(
          _objects[activeHandle]->getBoundary(),
          _objects[handle]->getBoundary(),
          intersectRect);
      if (found)  {
        _objects[activeHandle]->registerCollision(intersectRect);
        _objects[handle]->registerCollision(intersectRect);
      }
    }
  }
}

