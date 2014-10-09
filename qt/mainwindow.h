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

private:
    Ui::MainWindow *ui;
    void keyPressEvent(QKeyEvent *event);

    void liberarRecursos();
};

#endif // MAINWINDOW_H
