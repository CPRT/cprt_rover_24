#include "../include/camera_streaming_qt/mainwindow.h"

#include <QLabel>
#include <QPushButton>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  QPushButton* button = new QPushButton("Hello world");
  setCentralWidget(button);
}

MainWindow::~MainWindow() {}
