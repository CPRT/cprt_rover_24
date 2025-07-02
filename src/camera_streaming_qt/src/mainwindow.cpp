#include "mainwindow.h"

#include <QDebug>
#include <QLabel>

#include "widgets/main_widget.h"

MainWindow::MainWindow(CameraClient* camera_client, QWidget* parent)
    : QMainWindow(parent) {
  camera_client_ = camera_client;

  // Setup main widget
  MainWidget* main_widget = new MainWidget(this);

  connect(main_widget, &MainWidget::request_source_names, this,
          &MainWindow::get_source_names);

  setCentralWidget(main_widget);
}

MainWindow::~MainWindow() {}

void MainWindow::get_source_names(SourceWidget* source_widget) {
  if (!camera_client_) return;
  camera_client_->get_cameras();
}
