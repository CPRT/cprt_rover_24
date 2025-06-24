#ifndef PRESET_WIDGET_H
#define PRESET_WIDGET_H

#include <QWidget>

#include "widgets/source_widget.h"

class PresetWidget : public QWidget {
  Q_OBJECT

  public:
    PresetWidget(QWidget* parent = nullptr);
    ~PresetWidget();

  private:
    
};

#endif