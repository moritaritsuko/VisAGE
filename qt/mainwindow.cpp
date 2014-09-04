#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <VISAGE/prog.hpp>
#include <VISAGE/stereocams.hpp>
#include <VISAGE/conectrobo.hpp>

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <sched.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <fcntl.h>

#include<QKeyEvent>

unsigned int l = 9, a = 6, c = 0, p = 6008, r = 99;
float t = 25.4f;

Programa prog(l, a, t, c, p);
StereoCameras stereoCameras;
bool pararCap = false;

void uso()
{
    std::cerr << "-u   :  [U]so, imprime esta mensagem" << std::endl;
    std::cerr << "-l   :  [L]argura do tabuleiro (L >= 2)" << std::endl;
    std::cerr << "-a   :  [A]ltura do tabuleiro (A >= 2 & A != L)" << std::endl;
    std::cerr << "-t   :  [T]amanho do quadrado em milimetros (T >= 20.0)" << std::endl;
    std::cerr << "-c   :  [C]âmera a ser utilizada (C >= 0)" << std::endl;
    std::cerr << "-p   :  [P]orta para comunicação RSI (1024 <= P <= 32767)" << std::endl;
    std::cerr << "-r   :  P[r]ioridade (1 <= r <= 99)" << std::endl;
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    bool valoresPadrao = true;

    if (valoresPadrao)
    {
        l = 9u;
        a = 6u;
        t = 25.4f;
        c = 0u;
        p = 6008u;
        r = 99u;
    }
    else if ((l < 2u) || (a < 2u || a == l) || (t < 20.f) || (c < 0u) || (p < 1024u || p > 32767u) || (r < 1u || r > 99u))
    {
        uso();
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}




void MainWindow::on_btnRodar_clicked()
{


    /*struct sched_param sched;

    if(sched_getparam(0,&sched)==-1)
                perror("Falha em sched_getparam");

    sched.sched_priority = r;

    // int sched_setscheduler(pid_t pid, int policy, const struct  sched_param *p);
    if (sched_setscheduler(0, SCHED_FIFO, &sched) == -1)
    {
        perror("Falha em sched_setscheduler");
        exit(0);
    }

    if (sched_getparam(0, &sched) == -1)
        perror("Falha em sched_getparam");

    switch (sched_getscheduler(0))
    {
        case SCHED_FIFO:
                printf("\nScheduler: SCHED_FIFO. Priority: %d\n",sched.sched_priority);
                break;
        case SCHED_RR:
                printf("\nScheduler: SCHED_RR. Priority: %d\n",sched.sched_priority);
                break;
        case SCHED_OTHER:
                printf("\nScheduler: SCHED_OTHER. Priority: %d\n",sched.sched_priority);
                break;
    }*/

    try
    {

        cv::Mat imgMat;
        prog.mAproximando = false;
        while(true){
            prog.executar(imgMat);
            //cv::imshow("img",imgMat);
            cv::waitKey(3);
            if (!imgMat.empty()){
                cv::cvtColor(imgMat,imgMat,cv::COLOR_BGR2RGB);

                //cv::resize(imgMat,imgMat,cv::Size(imgMat.cols/3,imgMat.rows/3),0,0,cv::INTER_LINEAR);

                QImage image = QImage((uint8_t*) imgMat.data,imgMat.cols,imgMat.rows,imgMat.step,QImage::Format_RGB888);

                QPixmap pixma = QPixmap::fromImage(image);

                ui->lblExibirIMG->setPixmap(pixma);

                ui->lblExibirIMG->setFixedSize(pixma.size());
            }
        }
    }
    catch (std::exception& e)
    {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
    }

}

void MainWindow::on_btnManual_clicked()
{
    for(;;){
        prog.Manipular();
        if((cv::waitKey(3) & 255) == 27)
            break;
    }
}

void MainWindow::on_btnMover_clicked()
{
    double dx = ui->editDeltaX->text().toFloat();
    double dy = ui->editDeltaY->text().toFloat();
    double dz = ui->editDeltaZ->text().toFloat();
    prog.MoverPara(dx,dy,dz);
}

void MainWindow::on_btnRotacionar_clicked()
{
    double da = ui->editDeltaA->text().toFloat();
    double db = ui->editDeltaB->text().toFloat();
    double dc = ui->editDeltaC->text().toFloat();
    prog.Rotacionar(da,db,dc);
}

void MainWindow::keyPressEvent(QKeyEvent *event){

    //    if(event->key()==Qt::Key_W){
    //        ui->labelDir->setText("W");
    //    }

    //    if(event->key()==Qt::Key_A){
    //        ui->labelDir->setText("A");
    //    }

    //    if(event->key()==Qt::Key_D){
    //        ui->labelDir->setText("D");
    //    }

    //    if(event->key()==Qt::Key_S){
    //        ui->labelDir->setText("S");
    //    }
}

void MainWindow::on_btnStereo_clicked()
{
    pararCap = false;
    stereoCameras.exec();

    while (!pararCap)
    {
        stereoCameras.capture();
        auto photoPair = stereoCameras.getStereoPhotoPair();
        auto photo1 = photoPair->matPair.first;
        auto photo2 = photoPair->matPair.second;

        if (!photo1.empty() && !photo2.empty()){
            cv::waitKey(3);
            prog.mMensurium.Rodar("Cameras",photo1,photo2);


            cv::Mat matEx1(photo1.cols,photo1.rows,CV_8UC3);
            cv::cvtColor(photo1,matEx1,cv::COLOR_BGR2RGB);

            cv::resize(matEx1,matEx1,cv::Size(matEx1.cols/3,matEx1.rows/3),0,0,cv::INTER_LINEAR);

            QImage image = QImage((uint8_t*) matEx1.data,matEx1.cols,matEx1.rows,matEx1.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(image);

            ui->lblImgCamE->setPixmap(pixma);

            ui->lblImgCamE->setFixedSize(pixma.size());

            cv::Mat matEx2(photo1.cols,photo1.rows,CV_8UC3);
            cv::cvtColor(photo2,matEx2,cv::COLOR_BGR2RGB);

            cv::resize(matEx2,matEx2,cv::Size(matEx2.cols/3,matEx2.rows/3),0,0,cv::INTER_LINEAR);

            QImage imageD = QImage((uint8_t*) matEx2.data,matEx2.cols,matEx2.rows,matEx2.step,QImage::Format_RGB888);

            QPixmap pixmaD = QPixmap::fromImage(imageD);

            ui->lblImgCamD->setPixmap(pixmaD);

            ui->lblImgCamD->setFixedSize(pixmaD.size());
        }
        else{

            if (!photo1.empty() && !photo2.empty()){
                cv::waitKey(3);
                prog.mMensurium.Rodar("Cameras 1",photo1);


                cv::Mat matEx1(photo1.cols,photo1.rows,CV_8UC3);
                cv::cvtColor(photo1,matEx1,cv::COLOR_BGR2RGB);

                cv::resize(matEx1,matEx1,cv::Size(matEx1.cols/3,matEx1.rows/3),0,0,cv::INTER_LINEAR);

                QImage image = QImage((uint8_t*) matEx1.data,matEx1.cols,matEx1.rows,matEx1.step,QImage::Format_RGB888);

                QPixmap pixma = QPixmap::fromImage(image);

                ui->lblImgCamE->setPixmap(pixma);

                ui->lblImgCamE->setFixedSize(pixma.size());
            }

            if (!photo2.empty()){
                cv::waitKey(3);
                prog.mMensurium.Rodar("Camera 2",photo2);

                cv::Mat matEx2(photo1.cols,photo1.rows,CV_8UC3);
                cv::cvtColor(photo2,matEx2,cv::COLOR_BGR2RGB);

                cv::resize(matEx2,matEx2,cv::Size(matEx2.cols/3,matEx2.rows/3),0,0,cv::INTER_LINEAR);

                QImage imageD = QImage((uint8_t*) matEx2.data,matEx2.cols,matEx2.rows,matEx2.step,QImage::Format_RGB888);

                QPixmap pixmaD = QPixmap::fromImage(imageD);

                ui->lblImgCamD->setPixmap(pixmaD);

                ui->lblImgCamD->setFixedSize(pixmaD.size());

            }

        }


    }
    pararCap = false;
}

void MainWindow::on_btnGAMAGON_clicked()
{    
    prog.ativarGAMAG();
}

void MainWindow::on_btnGAMAGOFF_clicked()
{
    prog.desativarGAMAG();
}

bool salvar = false;
void MainWindow::on_btnCaptura_clicked()
{
    pararCap = false;
    stereoCameras.exec();
    int cont = 0;
    while(!pararCap){

        stereoCameras.capture();
        auto photoPair = stereoCameras.getStereoPhotoPair();
        auto photo1 = photoPair->matPair.first;
        auto photo2 = photoPair->matPair.second;



        cv::waitKey(5);
        if(ui->checkBoxSalvar->isChecked()) salvar = true;



        if (!photo1.empty()){
            cv::Mat matEx1(photo1.cols,photo1.rows,CV_8UC3);
            if(salvar) cv::imwrite("imgE"+std::to_string(cont)+".png",photo2);
            //             cv::imshow("photo1",matEx1);
            cv::cvtColor(photo1,matEx1,cv::COLOR_BGR2RGB);

            cv::resize(matEx1,matEx1,cv::Size(matEx1.cols/3,matEx1.rows/3),0,0,cv::INTER_LINEAR);

            QImage image = QImage((uint8_t*) matEx1.data,matEx1.cols,matEx1.rows,matEx1.step,QImage::Format_RGB888);

            QPixmap pixma = QPixmap::fromImage(image);

            ui->lblImgCamE->setPixmap(pixma);

            ui->lblImgCamE->setFixedSize(pixma.size());
        }

        if (!photo2.empty()){
            cv::Mat matEx2(photo1.cols,photo1.rows,CV_8UC3);
            if(salvar){
                cv::imwrite("imgD"+std::to_string(cont)+".png",photo2);
                cont++;
                salvar = false;
            }
            //cv::imshow("photo2",matEx2);
            cv::cvtColor(photo2,matEx2,cv::COLOR_BGR2RGB);

            cv::resize(matEx2,matEx2,cv::Size(matEx2.cols/3,matEx2.rows/3),0,0,cv::INTER_LINEAR);

            QImage imageD = QImage((uint8_t*) matEx2.data,matEx2.cols,matEx2.rows,matEx2.step,QImage::Format_RGB888);

            QPixmap pixmaD = QPixmap::fromImage(imageD);

            ui->lblImgCamD->setPixmap(pixmaD);

            ui->lblImgCamD->setFixedSize(pixmaD.size());


        }

    }
    pararCap = false;


}

void MainWindow::on_btnPararCap_clicked()
{
    pararCap = true;
    stereoCameras.stop();

}

void MainWindow::on_btnCalibStr_clicked()
{
    stereoCameras.showDisplayCapture();
    auto rms = stereoCameras.CalibrarStCam(25.f,cv::Size(9,6));
    std::cout << "Calibração concluída, RMS = " << rms << std::endl;
}

void MainWindow::on_btnDsip_clicked()
{
    pararCap = false;
    stereoCameras.exec();
    while(!pararCap){

        stereoCameras.capture();
        auto photoPair = stereoCameras.getStereoPhotoPair();
        auto photo1 = photoPair->matPair.first;
        auto photo2 = photoPair->matPair.second;

        if(!photo1.empty() && !photo2.empty())
            //prog.mMensurium.Stereo(photo1,photo2);
            prog.mMensurium.StereoOCL(photo1,photo2);

        cv::waitKey(3);
    }
}

void MainWindow::on_btnFoco_clicked()
{
    pararCap = false;
    stereoCameras.exec();
    while(!pararCap){

        stereoCameras.capture();
        auto photoPair = stereoCameras.getStereoPhotoPair();
        auto photo1 = photoPair->matPair.first;
        auto photo2 = photoPair->matPair.second;

        if(!photo1.empty() && !photo2.empty())
            prog.mMensurium.CalibrarFoco(photo1, photo2, cv::Size(9, 6), 4350);
        cv::waitKey(3);
    }

}


void MainWindow::on_btnSalvar_clicked()
{
    salvar = true;
}
