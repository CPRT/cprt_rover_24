#include "widgets/preset_widget.h"

PresetWidget::PresetWidget(QWidget* parent) : QWidget(parent) {
  main_layout_ = new QVBoxLayout(this);

  widget_header_ = new QLabel("Custom Preset");
  main_layout_->addWidget(widget_header_);

  // Create the layout and scroll area that will hold all the sources
  sources_container_ = new QWidget();
  sources_layout_ = new QVBoxLayout(sources_container_);
  sources_layout_->setAlignment(Qt::AlignTop);
  sources_container_->setLayout(sources_layout_);

  sources_scroll_area_ = new QScrollArea();
  sources_scroll_area_->setWidgetResizable(true);
  sources_scroll_area_->setWidget(sources_container_);

  main_layout_->addWidget(sources_scroll_area_);

  // Create default source
  sources_.push_back(new SourceWidget());
  sources_layout_->addWidget(sources_[0]);

  add_source_button_ = new QPushButton("Add Source");

  connect(add_source_button_, &QPushButton::clicked, this,
          &PresetWidget::add_source);

  main_layout_->addWidget(add_source_button_);

  submit_preset_button_ = new QPushButton("Submit Preset");

  connect(submit_preset_button_, &QPushButton::clicked, this,
          &PresetWidget::submit_preset);

  main_layout_->addWidget(submit_preset_button_);
}

PresetWidget::~PresetWidget() {}

void PresetWidget::add_source() {
  if (!sources_layout_) return;

  SourceWidget* src = new SourceWidget();
  sources_.push_back(src);
  sources_layout_->addWidget(src);
}

void PresetWidget::submit_preset() {}