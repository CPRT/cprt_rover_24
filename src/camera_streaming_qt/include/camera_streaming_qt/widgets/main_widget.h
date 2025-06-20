#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

class MainWidget : public QWidget {
  Q_OBJECT

 public:
  MainWidget(QWidget* parent = nullptr);
  ~MainWidget();

 public slots:
  void set_signal_server_ip(QString ip);

 private:
  QVBoxLayout* main_layout_;

  // Presets UI
  QHBoxLayout* preset_layout_;
  QPushButton* drive_preset_button_;
  QPushButton* eef_preset_button_;
  QPushButton* microscope_preset_button_;
  QPushButton* belly_preset_button_;
  QPushButton* drive_eef_preset_button_;
  QPushButton* eef_drive_preset_button_;

  // Signal server UI
  QHBoxLayout* signal_server_layout_;
  QLineEdit* server_ip_line_edit_;
  QPushButton* server_connect_button_;

  QString signal_server_ip_;
};

#endif