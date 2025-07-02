#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "camera_client.h"
#include "ros_client.h"
#include "widgets/source_widget.h"

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(CameraClient* camera_client = nullptr, QWidget* parent = nullptr);
  ~MainWindow();

 signals:
  void request_source_names(SourceWidget* source_widget);

 private slots:
  void get_source_names(SourceWidget* source_widget);

 private:
  CameraClient* camera_client_;
};
#endif  // MAINWINDOW_H
