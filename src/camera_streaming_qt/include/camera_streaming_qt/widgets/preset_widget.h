/**
 * @file preset_widget.h
 * @brief Header file for the PresetWidget class.
 * @author Aria Wong
 *
 * This file contains the declaration of the PresetWidget class, which
 * is holds all of the widgets that lets a user customize a preset.
 */

#ifndef PRESET_WIDGET_H
#define PRESET_WIDGET_H

#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>
#include <vector>

#include "models/source_model.h"
#include "widgets/source_widget.h"

/**
 * @class PresetWidget
 * @brief A class that holds the widgets to allow a user to customize a preset.
 *
 * The PresetWidget class holds the widgets that let the user customize a
 * preset, mainly the list of sources.
 */
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

  QLabel* widget_header_;

  QWidget* sources_container_;
  QVBoxLayout* sources_layout_;
  QScrollArea* sources_scroll_area_;
  QVBoxLayout* main_layout_;

  QPushButton* add_source_button_;
  QPushButton* submit_preset_button_;
};

#endif