#ifndef PRESET_WIDGET_H
#define PRESET_WIDGET_H

#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>
#include <vector>

#include "models/source_model.h"
#include "widgets/source_widget.h"

class PresetWidget : public QWidget {
  Q_OBJECT

 public:
  PresetWidget(QWidget* parent = nullptr);
  ~PresetWidget();

 public slots:
  void add_source();
  void submit_preset();

 private:
  std::vector<SourceWidget*> sources_;

  QWidget* sources_container_;
  QVBoxLayout* sources_layout_;
  QScrollArea* sources_scroll_area_;
  QVBoxLayout* main_layout_;

  QPushButton* add_source_button_;
  QPushButton* submit_preset_button_;
};

#endif