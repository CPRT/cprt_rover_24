#include "mainwindow.h"

#include <QLabel>

#include "widgets/main_widget.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  // Setup main widget
  MainWidget* main_widget_ = new MainWidget(this);
  setCentralWidget(main_widget_);
}

MainWindow::~MainWindow() {}
