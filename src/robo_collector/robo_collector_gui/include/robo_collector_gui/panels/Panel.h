#ifndef ROBO_COLLECTOR_GUI_PANELS_PANEL_H_
#define ROBO_COLLECTOR_GUI_PANELS_PANEL_H_

//C system headers

//C++ system headers
#include <array>

//Other libraries headers
#include "manager_utils/drawing/Image.h"

//Own components headers
#include "robo_collector_gui/panels/config/PanelConfig.h"

//Forward declarations

class Panel {
public:
  int32_t init(const PanelConfig& cfg);
  void draw() const;

  void shrinkHealthIndicator(int32_t deltaPx);

private:
  enum InternalDefines {
    TIME_PANEL,
    COIN_PANEL,
    HEALTH_PANEL,

    PANELS_CTN
  };

  std::array<Image, PANELS_CTN> _panels;

  Image _healthIndicator;
  Image _horDelimiter;
  Image _vertDelimiter;
};

#endif /* ROBO_COLLECTOR_GUI_PANELS_PANEL_H_ */
