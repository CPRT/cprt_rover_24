#include "mainwindow.h"

#include <QDebug>
#include <QLabel>

#include "widgets/main_widget.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  // Setup ROS client
  ros_client_ = new ROSClient();

  // Setup main widget
  MainWidget* main_widget = new MainWidget(this);

  connect(main_widget, &MainWidget::request_source_names, this,
          &MainWindow::get_cameras);

  setCentralWidget(main_widget);
}

MainWindow::~MainWindow() {
  if (ros_client_) {
    delete ros_client_;
  }
}

void MainWindow::get_cameras() {
  qDebug() << "4";
  ros_client_->get_cameras();
}
