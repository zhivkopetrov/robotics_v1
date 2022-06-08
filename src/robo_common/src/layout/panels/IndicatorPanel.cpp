//Corresponding header
#include "robo_common/layout/panels/IndicatorPanel.h"

//System headers
#include <sstream>

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

namespace {
template <typename T>
std::string toStringWithPrecision(const T value, const int32_t precision = 6) {
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed << value;
  return out.str();
}
}

ErrorCode IndicatorPanel::init(const IndicatorPanelConfig &cfg,
                               const IndicatorPanelUtilityConfig &utilityCfg) {
  _indicatorModifyTimerId = cfg.indicatorModifyTimerId;

  if (nullptr == utilityCfg.indicatorDepletedCb) {
    LOGERR("Error, nullptr provided for IndicatorDepletedCb");
    return ErrorCode::FAILURE;
  }
  _indicatorDepletedCb = utilityCfg.indicatorDepletedCb;

  _panel.create(cfg.rsrcId);
  _panel.setPosition(utilityCfg.pos);

  _indicator.create(cfg.indicatorRsrcId);
  if (INDICATOR_PANEL_MAX_VALUE != _indicator.getImageWidth()) {
    LOGERR("Error, INDICATOR_PANEL_MAX_VALUE (%d) should be equal to "
           "_indicator image width (%d)", INDICATOR_PANEL_MAX_VALUE,
           _indicator.getImageWidth());
    return ErrorCode::FAILURE;
  }

  _indicator.setPosition(utilityCfg.pos.x + 79, utilityCfg.pos.y + 13);
  _indicator.setCropRect(_indicator.getImageRect());

  _indicatorText.create(cfg.indicatorFontId, "100%", Colors::RED);
  setAndCenterIndicatorText();

  return ErrorCode::SUCCESS;
}

void IndicatorPanel::draw() const {
  _panel.draw();
  _indicator.draw();
  _indicatorText.draw();
}

void IndicatorPanel::modifyIndicator(int32_t delta) {
  _animTicksLeft += delta;

  const bool isAnimationActive = isActiveTimerId(_indicatorModifyTimerId);
  if (0 == _animTicksLeft) {
    if (isAnimationActive) {
      stopTimer(_indicatorModifyTimerId);
    }
    return; //nothing more to do
  }

  if (0 < _animTicksLeft) {
    _currAnimType = IndicatorPanelAnimationType::INCREASE;
  } else {
    _currAnimType = IndicatorPanelAnimationType::DECREASE;
  }

  if (!isAnimationActive) {
    constexpr auto timerInterval = 20;
    startTimer(timerInterval, _indicatorModifyTimerId, TimerType::PULSE);
  }
}

int32_t IndicatorPanel::getIndicatorValue() const {
  return _indicator.getCroppedFrameWidth();
}

void IndicatorPanel::onTimeout(const int32_t timerId) {
  if (timerId == _indicatorModifyTimerId) {
    if (IndicatorPanelAnimationType::INCREASE == _currAnimType) {
      processIndicatorIncreaseAnim();
    } else {
      processIndicatorReduceAnim();
    }
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void IndicatorPanel::processIndicatorIncreaseAnim() {
  const Point absPos = _indicator.getPosition();
  Rectangle cropRectangle = _indicator.getCropRect();

  //dimension needs to be overridden, because
  //crop rectangle::ZERO will zero out the crop rectangle position
  cropRectangle.x = absPos.x;
  cropRectangle.y = absPos.y;
  cropRectangle.h = _indicator.getFrameHeight();
  auto &remainingIndicator = cropRectangle.w;
  if (INDICATOR_PANEL_MAX_VALUE == remainingIndicator) {
    _animTicksLeft = 0;
    stopTimer(_indicatorModifyTimerId);
    //indicatorFullCb() could be added here
    return;
  }

  if (0 == _animTicksLeft) {
    stopTimer(_indicatorModifyTimerId);
    return;
  }
  --_animTicksLeft;

  ++remainingIndicator;

  _indicator.setCropRect(cropRectangle);
  setAndCenterIndicatorText();
}

void IndicatorPanel::processIndicatorReduceAnim() {
  Rectangle cropRectangle = _indicator.getCropRect();
  auto &remainingIndicator = cropRectangle.w;
  if (0 == remainingIndicator) {
    _animTicksLeft = 0;
    stopTimer(_indicatorModifyTimerId);
    _indicatorDepletedCb();
    return;
  }

  if (0 == _animTicksLeft) {
    stopTimer(_indicatorModifyTimerId);
    return;
  }
  ++_animTicksLeft;

  --remainingIndicator;
  _indicator.setCropRect(cropRectangle);
  setAndCenterIndicatorText();
}

void IndicatorPanel::setAndCenterIndicatorText() {
  const int32_t totalIndicator = _indicator.getImageWidth();
  const Rectangle indicatorCropRect = _indicator.getCropRect();
  const int32_t remainingIndicator = indicatorCropRect.w;
  const double remainingIndicatorPecent = (remainingIndicator * 100.0)
      / totalIndicator;
  constexpr int32_t precision = 1;
  const std::string textContent =
      toStringWithPrecision(remainingIndicatorPecent, precision) + "%";

  _indicatorText.setText(textContent.c_str());

  auto widgetAlignArea = indicatorCropRect;
  if (0 == remainingIndicatorPecent) {
    widgetAlignArea = _indicator.getImageRect();
  } else if (remainingIndicatorPecent < 50) {
    widgetAlignArea.x += widgetAlignArea.w;
    widgetAlignArea.w = totalIndicator - remainingIndicator;
    _indicatorText.setColor(Colors::GREEN);
  } else {
    _indicatorText.setColor(Colors::RED);
  }

  const auto indicatorTextWidth = _indicatorText.getImageWidth();
  const auto textPos = WidgetAligner::getPosition(indicatorTextWidth,
      _indicatorText.getImageHeight(), widgetAlignArea,
      WidgetAlignment::CENTER_CENTER);
  _indicatorText.setPosition(textPos);
}
