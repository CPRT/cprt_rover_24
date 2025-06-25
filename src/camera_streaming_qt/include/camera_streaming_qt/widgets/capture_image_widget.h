#ifndef CAPTURE_IMAGE_WIDGET_H
#define CAPTURE_IMAGE_WIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>

class CaptureImageWidget : public QWidget {
  Q_OBJECT

  public:
    CaptureImageWidget(QWidget* parent = nullptr);
    ~CaptureImageWidget();

  public slots:
    void set_source_name(QString name);
    void set_filename(QString filename);

  private:
    QVBoxLayout* main_layout_;    

    QLabel* widget_header_;

    // Source name and refresh button
    QString source_name_;
    QHBoxLayout* source_layout_;
    QLabel* source_name_label_;
    QComboBox* source_name_combo_box_;
    QPushButton* refresh_button_;

    // Filename
    QString filename_;
    QHBoxLayout* filename_layout_;
    QLabel* filename_label_;
    QLineEdit* filename_line_edit_;

    QPushButton* capture_image_button_;
};

#endif