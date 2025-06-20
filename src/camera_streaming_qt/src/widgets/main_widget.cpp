#include "widgets/main_widget.h"

#include <QDebug>

MainWidget::MainWidget(QWidget* parent) : QWidget(parent) {
  main_layout_ = new QVBoxLayout(this);

  // Setup presets UI
  preset_layout_ = new QHBoxLayout();

  drive_preset_button_ = new QPushButton("Drive");
  preset_layout_->addWidget(drive_preset_button_);

  eef_preset_button_ = new QPushButton("EEF");
  preset_layout_->addWidget(eef_preset_button_);

  microscope_preset_button_ = new QPushButton("Microscope");
  preset_layout_->addWidget(microscope_preset_button_);

  belly_preset_button_ = new QPushButton("Belly");
  preset_layout_->addWidget(belly_preset_button_);

  drive_eef_preset_button_ = new QPushButton("Drive + EEF");
  preset_layout_->addWidget(drive_eef_preset_button_);

  eef_drive_preset_button_ = new QPushButton("EEF + Drive");
  preset_layout_->addWidget(eef_drive_preset_button_);

  main_layout_->addLayout(preset_layout_);

  // Setup signal server UI
  signal_server_layout_ = new QHBoxLayout();

  server_ip_line_edit_ = new QLineEdit("ws://localhost:8443");
  signal_server_layout_->addWidget(server_ip_line_edit_);

  server_connect_button_ = new QPushButton("Connect");
  signal_server_layout_->addWidget(server_connect_button_);

  connect(server_ip_line_edit_, &QLineEdit::textChanged, this,
          &MainWidget::set_signal_server_ip);

  main_layout_->addLayout(signal_server_layout_);

  temp_source_ = new SourceWidget();
  main_layout_->addWidget(temp_source_);

  // Make widgets start at top
  main_layout_->setAlignment(Qt::AlignTop);
}

MainWidget::~MainWidget() {}

void MainWidget::set_signal_server_ip(QString ip) {
  signal_server_ip_ = ip;
  qDebug() << "IP: " << signal_server_ip_;
}