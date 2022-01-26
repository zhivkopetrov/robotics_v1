//Corresponding header
#include "robo_collector_gui/helpers/CollisionWatcher.h"

//C system headers

//C++ system headers
#include <algorithm>

//Other libraries headers
#include "sdl_utils/drawing/GeometryUtils.h"
#include "utils/Log.h"

//Own components headers

CollisionObjHandle CollisionWatcher::registerObject(
    CollisionObject *object, CollisionDamageImpact impact) {
  CollisionObjHandle handleIdx = 0;
  for (auto &data : _objects) {
    // free index found, occupy it
    if (nullptr == data.object) {
      data.object = object;
      data.impact = impact;
      return handleIdx;
    }

    ++handleIdx;
  }

  _objects.emplace_back(object, impact);
  return handleIdx;
}

void CollisionWatcher::unregisterObject(CollisionObjHandle handle) {
  if (handle >= _objects.size()) {
    LOGERR("CollisionObjHandle: %d not found", handle);
    return;
  }

  _objects[handle].object = nullptr;
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
    const auto& activeData = _objects[activeHandle];

    for (CollisionObjHandle handle = 0; handle < objSize; ++handle) {
      if (handle == activeHandle) {
        continue; //skip self-collision
      }
      const auto& checkedData = _objects[handle];

      const bool found = GeometryUtils::findRectIntersection(
          activeData.object->getBoundary(),
          checkedData.object->getBoundary(),
          intersectRect);
      if (found) {
        checkedData.object->registerCollision(intersectRect, activeData.impact);

        //process the active data last, because
        //it might end the turn immediately
        activeData.object->registerCollision(intersectRect, checkedData.impact);
      }
    }
  }
}

