#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>

class MainWidget : public QWidget {
  Q_OBJECT

  public:
    MainWidget(QWidget* parent = nullptr); 
    ~MainWidget();

  public slots:
    void set_signal_server_ip(QString ip);

  private:
    QVBoxLayout* main_layout_;
    
    // Signal server UI
    QHBoxLayout* signal_server_layout_;
    QLineEdit* server_ip_line_edit_;
    QPushButton* server_connect_button_; 

    QString signal_server_ip_;
};

#endif 