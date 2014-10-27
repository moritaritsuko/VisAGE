#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_btnRodar_clicked();

    void on_btnManual_clicked();

    void on_btnMover_clicked();

    void on_btnStereo_clicked();

    void on_btnRotacionar_clicked();

    void on_btnGAMAGON_clicked();

    void on_btnGAMAGOFF_clicked();

    void on_btnCaptura_clicked();

    void on_btnPararCap_clicked();

    void on_btnCalibStr_clicked();

    void on_btnDsip_clicked();

    void on_btnFoco_clicked();

    void on_btnSalvar_clicked();

    void on_btninVision_clicked();

    void on_btnIV_2_clicked();

    void on_btnCapturaMono_clicked();

    void on_btnTesteCor_clicked();

    void on_horizontalSlider_H_sliderMoved(int position);

    void on_horizontalSlider_S_sliderMoved(int position);

    void on_horizontalSlider_V_sliderMoved(int position);

    void on_radioButton_G_clicked();

    void on_radioButton_R_clicked();

    void on_radioButton_Y_clicked();

    void on_radioButton_B_clicked();

    void on_horizontalSlider_H_f_sliderMoved(int position);

    void on_horizontalSlider_S_f_sliderMoved(int position);

    void on_horizontalSlider_V_f_actionTriggered(int action);

    void on_horizontalSlider_V_f_sliderMoved(int position);

    void on_btnCapIV_clicked();

    void on_vtnCalibIV_clicked();

    void on_horizontalSlider_H_valueChanged(int value);

    void on_horizontalSlider_S_valueChanged(int value);

    void on_horizontalSlider_V_valueChanged(int value);

    void on_horizontalSlider_H_f_valueChanged(int value);

    void on_horizontalSlider_S_f_valueChanged(int value);

    void on_horizontalSlider_V_f_valueChanged(int value);

private:
    Ui::MainWindow *ui;
    void keyPressEvent(QKeyEvent *event);

    void liberarRecursos();

};

#endif // MAINWINDOW_H
