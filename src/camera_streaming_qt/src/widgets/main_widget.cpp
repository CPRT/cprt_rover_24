#include <QDebug>
#include "widgets/main_widget.h"

MainWidget::MainWidget(QWidget* parent) 
  : QWidget(parent) {
  main_layout_ = new QVBoxLayout(this);

  // Setup signal server UI
  signal_server_layout_ = new QHBoxLayout();

  server_ip_line_edit_ = new QLineEdit("ws://localhost:8443");
  signal_server_layout_->addWidget(server_ip_line_edit_);

  server_connect_button_ = new QPushButton("Connect");
  signal_server_layout_->addWidget(server_connect_button_);

  connect(server_ip_line_edit_, &QLineEdit::textChanged, this, &MainWidget::set_signal_server_ip);

  main_layout_->addLayout(signal_server_layout_);
}

MainWidget::~MainWidget() {

}

void MainWidget::set_signal_server_ip(QString ip) {
  signal_server_ip_ = ip;
  qDebug() << "IP: " << signal_server_ip_;
}