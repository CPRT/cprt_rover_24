#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <interfaces/msg/video_source.hpp>
#include <string>
#include <vector>

#include "camera_client.h"
#include "widgets/main_widget.h"
#include "widgets/source_widget.h"

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(CameraClient* camera_client = nullptr, QWidget* parent = nullptr);
  ~MainWindow();

 signals:
  void request_source_names();

 private slots:
  void get_source_names();
  void receive_preset(std::vector<interfaces::msg::VideoSource> preset);

 private:
  CameraClient* camera_client_;
  MainWidget* main_widget_;
};
#endif  // MAINWINDOW_H
