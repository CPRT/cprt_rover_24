#include "widgets/source_widget.h"

SourceWidget::SourceWidget(QWidget* parent) 
  : QWidget(parent) {
  main_layout_ = new QVBoxLayout(this);    

  source_name_label_ = new QLabel("Source");
  main_layout_->addWidget(source_name_label_);

  // Setup name UI
  name_layout_ = new QHBoxLayout();
  
  name_label_ = new QLabel("Name: ");
  name_layout_->addWidget(name_label_);

  name_combo_box_ = new QComboBox();
  name_layout_->addWidget(name_combo_box_);

  refresh_sources_button_ = new QPushButton("Refresh");
  name_layout_->addWidget(refresh_sources_button_);

  main_layout_->addLayout(name_layout_);

  // Setup size UI
  size_layout_ = new QHBoxLayout();

  size_validator_ = new QIntValidator();
  size_validator_->setBottom(0);

  width_label_ = new QLabel("Width: ");
  size_layout_->addWidget(width_label_);

  width_line_edit_ = new QLineEdit(QString::number(100));
  width_line_edit_->setValidator(size_validator_);
  size_layout_->addWidget(width_line_edit_);

  height_label_ = new QLabel("Height: ");
  size_layout_->addWidget(height_label_);

  height_line_edit_ = new QLineEdit(QString::number(100));
  height_line_edit_->setValidator(size_validator_);
  size_layout_->addWidget(height_line_edit_);

  main_layout_->addLayout(size_layout_);

  // Setup origin UI
  origin_layout_ = new QHBoxLayout();

  origin_validator_ = new QIntValidator();

  origin_x_label_ = new QLabel("Origin X: ");
  origin_layout_->addWidget(origin_x_label_);

  origin_x_line_edit_ = new QLineEdit(QString::number(0));
  origin_x_line_edit_->setValidator(origin_validator_);
  origin_layout_->addWidget(origin_x_line_edit_);

  origin_y_label_ = new QLabel("Origin Y: ");
  origin_layout_->addWidget(origin_y_label_);

  origin_y_line_edit_ = new QLineEdit(QString::number(0));
  origin_y_line_edit_->setValidator(origin_validator_);
  origin_layout_->addWidget(origin_y_line_edit_);

  main_layout_->addLayout(origin_layout_);

  // Setup remove button
  remove_button_ = new QPushButton("Remove Source");
  main_layout_->addWidget(remove_button_);
}

SourceWidget::~SourceWidget() {

}