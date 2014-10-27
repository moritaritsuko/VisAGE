#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QTime>
#include <QMessageBox>

#include <VISAGE/prog.hpp>
#include <VISAGE/stereocams.hpp>
#include <VISAGE/conectrobo.hpp>
#include <VISAGE/calibracam.hpp>

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

#include "camera.hpp"
#include <iostream>
#include <fstream>

#include <iostream>
#include <boost/asio.hpp>

#include "gigabit_devcomm/image_data.hpp"
#include "gigabit_devcomm/configure_camera.hpp"
#include "boost/thread/thread.hpp"
#include  "boost/bind.hpp"

#include<zbar.h>


unsigned int l = 9, a = 6, c = 0, p = 6008, r = 99;
float t = 25.4f;
cv::Scalar corI[4] =    {cv::Scalar(20,20,90),   cv::Scalar(0,20,90),   cv::Scalar(30,20,90),  cv::Scalar(40,10,20)};
cv::Scalar corF[4] =    {cv::Scalar(30,250,250), cv::Scalar(20,250,250), cv::Scalar(40,250,250), cv::Scalar(80,250,250)};

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

cv::Mat imageIVison(int ganho = 32,int ganhoAn = 8,int exp = 3840, int offS = 2840){

    std::cout<<"Aguardando imagem..."<<std::endl;

    size_t width = 5120;
    size_t height = 3840;
    std::string ip = "192.168.0.37";

    ivsn::devcomm::ConfigureCamera config(ip);

    config.updateSensorGain(ganho);
    config.updateSensorAnalogGain(ganhoAn);
    config.updateSensorExposureTime(exp);
    config.updateSensorOffset(offS);

    boost::asio::mutable_buffer buffer;
    ivsn::devcomm::ImageData image;
    buffer = image.getImage(ip);

    std::size_t bufferSize = boost::asio::buffer_size(buffer);

    if (bufferSize != 0)
    {
        cv::Mat_<uint16_t> image(height, width);
        image.data = (uint8_t*) boost::asio::buffer_cast<uint16_t*>(buffer);
        cv::imwrite("image.png", image);

        cv::FileStorage fs("image.yml", cv::FileStorage::WRITE);

            fs << "image" << image ;

            fs.release();


        cv::Mat bayer8BitMat;
        image.convertTo(bayer8BitMat, CV_16UC1, 16);
        cv::imwrite("bayer8BitMat.png", bayer8BitMat);

        bayer8BitMat.convertTo(bayer8BitMat, CV_8UC1, 1.0/64);
        cv::imwrite("bayer8BitMat-2.png", bayer8BitMat);

        cv::Mat_<cv::Vec3b> img(image.size());
        cv::cvtColor(bayer8BitMat, img, CV_BayerBG2BGR);
        cv::imwrite("imageToShow.png", img);
        std::cout<<"Imagem adquirida!"<<std::endl;

        return img;

    }
    else
    {
        std::cout << "Imagem vazia." << std::endl;
        return cv::Mat();
    }


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
        for(int i = 0;i<4;++i){ prog.mMensurium.setarCores(corI[i],corF[i],i);}
    }
    else if ((l < 2u) || (a < 2u || a == l) || (t < 20.f) || (c < 0u) || (p < 1024u || p > 32767u) || (r < 1u || r > 99u))
    {
        uso();
    }
}

MainWindow::~MainWindow()
{
    liberarRecursos();
    delete ui;
}

void MainWindow::on_btnRodar_clicked()
{
    prog.IniciarCaptura();

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
    prog.IniciarCaptura();
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

void MainWindow::on_btnCapturaMono_clicked()
{
    pararCap = false;
    prog.IniciarCaptura();

    while (!pararCap)
    {
        prog.CapturaCameraMono();

    }
}


void MainWindow::on_btnPararCap_clicked()
{
    liberarRecursos();
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

void MainWindow::on_btninVision_clicked()
{
    Camera camera(std::string("192.168.0.37"), 13000);
    boost::asio::mutable_buffer imgBuffer = camera.capture();
    size_t bufferSize = boost::asio::buffer_size(imgBuffer);
    uint16_t* buffer = boost::asio::buffer_cast<uint16_t*>(imgBuffer);
    size_t width = camera.getWidth();
    size_t height = camera.getHeight();
    std::ofstream imageOutputStream("outputImage.pgm");
    imageOutputStream << "P2\n#Output\n" << width << " " << height << "\n4095\n";
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            uint16_t pixel = buffer[i*width+j];
            imageOutputStream << pixel << " ";
        }
        imageOutputStream << "\n";
    }
    imageOutputStream.close();
}

void MainWindow::on_btnIV_2_clicked()
{
do{
    cv::Mat_<cv::Vec3b> imageToShow = imageIVison(ui->spinBoxGain->value(),ui->spinBoxGainAn->value(),ui->spinBoxExp->value(),ui->spinBoxOffS->value());

        if (!imageToShow.empty())
        {


            cv::Mat imgPB;
            cv::Mat pbHSV;
            cv::cvtColor(imageToShow,imgPB,CV_BGR2GRAY);

            //cv::cvtColor(imageToShow,imgHSV,CV_BGR2HSV);
            //cv::inRange(imgHSV,cv::Scalar(0,0,0),cv::Scalar(255,100,50),pbHSV);
            //cv::threshold(pbHSV,pbHSV,5,255 , CV_THRESH_BINARY_INV);// 3 = 70 =72
            cv::adaptiveThreshold(imgPB,pbHSV,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,121,17);


            cv::resize(pbHSV,pbHSV,cv::Size(imgPB.cols/4,imgPB.rows/4));

            cv::imshow("pbHSV", pbHSV);

            CvMat** trans;

            cv::Mat imgSaida;
            imageToShow.copyTo(imgSaida);

            //prog.mMensurium.AcharTabs(imageToShow,4,trans,0,imgSaida);

            prog.mMensurium.Rodar("SaidaMe",imgSaida);

            //Se possuir interface grafica
            cv::resize(imgSaida,imgSaida,cv::Size(imgSaida.cols/4,imgSaida.rows/4));
            cv::namedWindow("imgSaida");
            cv::imshow("imgSaida", imgSaida);
            if(ui->checkBoxSalIV->isChecked()) cv::imwrite("saida.jpg",imageToShow);

            zbar::ImageScanner scanner;
            scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

            cv::Mat grey;
            cv::cvtColor(imageToShow,grey,CV_BGR2GRAY);
            int width = imageToShow.cols;
            int height = imageToShow.rows;
            uchar *raw = (uchar *)grey.data;
            // wrap image data
            zbar::Image image(width, height, "Y800", raw, width * height);
            // scan the image for barcodes
            int n = scanner.scan(image);

            // extract results
            if (n != 0)
            {
                for(zbar::Image::SymbolIterator symbol = image.symbol_begin();
                symbol != image.symbol_end();
                ++symbol)
                {
                    std::vector<cv::Point> vp;
                // do something useful with results
                std::cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' <<" "<< std::endl;
                  int n = symbol->get_location_size();
                  for(int i=0;i<n;i++)
                  {
                    vp.push_back(cv::Point(symbol->get_location_x(i),symbol->get_location_y(i)));
                  }
                  cv::RotatedRect r = cv::minAreaRect(vp);
                         cv::Point2f pts[4];
                         r.points(pts);
                         for(int i=0;i<4;i++){
                           cv::line(imageToShow,pts[i],pts[(i+1)%4],cv::Scalar(255,0,0),3);
                         }
                         //cout<<"Angle: "<<r.angle<<endl;
                 }
            }
            else std::cout << "Nada encontrado" << std::endl;


            cv::waitKey(50);
        }
        else
        {
            std::cout << "Imagem vazia." << std::endl;
        }
    }while (ui->checkBoxContIV->isChecked());


    std::cout << "IVISION: Finalizando Gigabit DevComm Test" << std::endl;
}

void MainWindow::liberarRecursos()
{
    std::cout << "Liberando recursos..." << std::endl;
    pararCap = true;
    stereoCameras.stop();
    if (prog.cap.isOpened())
        prog.cap.release();
}

cv::Mat img20MP;

void MainWindow::on_btnTesteCor_clicked()
{

    pararCap = false;

    //while(!pararCap){

    QTime t;
    t.start();
    cv::Mat img;
    if(ui->checkBoxVivo->isChecked()){ img = imageIVison();img20MP = img;}else{img = img20MP;} // If "Vivo" is checked, work with freshly captured image, otherwise work from previous capture
    std::cout<<"Tempo de aquisição: "<<t.restart()<<std::endl;

    if(!img.empty()){

        cv::Mat matHSV(img.rows,img.cols,img.type());
        cv::cvtColor(img,matHSV,CV_BGR2HSV); //Image conversion from GBR to HSV space

        cv::Scalar corIl(ui->horizontalSlider_H->value()  ,ui->horizontalSlider_S->value()  ,ui->horizontalSlider_V->value()); // Lower color bounds acquired from sliders
        cv::Scalar corFl(ui->horizontalSlider_H_f->value(),ui->horizontalSlider_S_f->value(),ui->horizontalSlider_V_f->value()); // Upper color bounds acquired from sliders

        cv::Mat matTh(img.rows,img.cols,CV_8UC1); // Container for output image
        cv::inRange(matHSV,corIl,corFl,matTh); // Output image generated for ranges given

        cv::resize(img,img,cv::Size(img.cols/4,img.rows/4)); //Image is resized to permitted dimensions
        cv::resize(matTh,matTh,cv::Size(matTh.cols/4,matTh.rows/4));

        if(ui->radioButton_Y->isChecked()){
            prog.mMensurium.setarCores(corIl,corFl,0);

        }

        if(ui->radioButton_R->isChecked()){
            prog.mMensurium.setarCores(corIl,corFl,1);
        }

        if(ui->radioButton_G->isChecked()){
            prog.mMensurium.setarCores(corIl,corFl,2);
        }

        if(ui->radioButton_B->isChecked()){
            prog.mMensurium.setarCores(corIl,corFl,3);
        }


        cv::imshow("img",img);
        cv::imshow("matTh",matTh);
    }
    else {
        std::cout << "Imagem vazia." << std::endl;
    }

    cv::waitKey(100);
    // }

}

void MainWindow::on_horizontalSlider_H_sliderMoved(int position)
{
    ui->label_H_Value->setText(QString::number(position));
}


void MainWindow::on_horizontalSlider_S_sliderMoved(int position)
{
    ui->horizontalSlider_S->setToolTip(QString::number(ui->horizontalSlider_S->value()));
    ui->label_S_Value->setText(QString::number(position));
}

void MainWindow::on_horizontalSlider_V_sliderMoved(int position)
{
    ui->horizontalSlider_V->setToolTip(QString::number(ui->horizontalSlider_V->value()));
    ui->label_V_Value->setText(QString::number(position));
}

void MainWindow::on_radioButton_G_clicked()
{
    ui->horizontalSlider_H->setValue(corI[2][0]);ui->horizontalSlider_H_f->setValue(corF[2][0]);
    ui->horizontalSlider_S->setValue(corI[2][1]);ui->horizontalSlider_S_f->setValue(corF[2][1]);
    ui->horizontalSlider_V->setValue(corI[2][2]);ui->horizontalSlider_V_f->setValue(corF[2][2]);

}

void MainWindow::on_radioButton_R_clicked()
{
    ui->horizontalSlider_H->setValue(corI[1][0]);ui->horizontalSlider_H_f->setValue(corF[1][0]);
    ui->horizontalSlider_S->setValue(corI[1][1]);ui->horizontalSlider_S_f->setValue(corF[1][1]);
    ui->horizontalSlider_V->setValue(corI[1][2]);ui->horizontalSlider_V_f->setValue(corF[1][2]);

}

void MainWindow::on_radioButton_Y_clicked()
{
    ui->horizontalSlider_H->setValue(corI[0][0]);ui->horizontalSlider_H_f->setValue(corF[0][0]);
    ui->horizontalSlider_S->setValue(corI[0][1]);ui->horizontalSlider_S_f->setValue(corF[0][1]);
    ui->horizontalSlider_V->setValue(corI[0][2]);ui->horizontalSlider_V_f->setValue(corF[0][2]);
}

void MainWindow::on_radioButton_B_clicked()
{
    ui->horizontalSlider_H->setValue(corI[3][0]);ui->horizontalSlider_H_f->setValue(corF[3][0]);
    ui->horizontalSlider_S->setValue(corI[3][1]);ui->horizontalSlider_S_f->setValue(corF[3][1]);
    ui->horizontalSlider_V->setValue(corI[3][2]);ui->horizontalSlider_V_f->setValue(corF[3][2]);
}

void MainWindow::on_horizontalSlider_H_f_sliderMoved(int position)
{
    ui->horizontalSlider_H_f->setToolTip(QString::number(ui->horizontalSlider_H_f->value()));
    ui->label_H_Value_f->setText(QString::number(position));
}

void MainWindow::on_horizontalSlider_S_f_sliderMoved(int position)
{
    ui->horizontalSlider_S_f->setToolTip(QString::number(ui->horizontalSlider_S_f->value()));
    ui->label_S_Value_f->setText(QString::number(position));
}

void MainWindow::on_horizontalSlider_V_f_sliderMoved(int position)
{
    ui->horizontalSlider_V_f->setToolTip(QString::number(ui->horizontalSlider_V_f->value()));
    ui->label_V_Value_f->setText(QString::number(position));
}

void MainWindow::on_btnCapIV_clicked()
{

    do{
        QTime t;
        t.start();
        cv::Mat img = imageIVison(ui->spinBoxGain->value(),ui->spinBoxGainAn->value(),ui->spinBoxExp->value(),ui->spinBoxOffS->value());
        std::cout<<"Tempo de aquisição: "<<t.restart()<<std::endl;

        cv::resize(img,img,cv::Size(img.cols/ui->spinBoxFator->value(),img.rows/ui->spinBoxFator->value()));

        cv::imshow("Imagem Saida",img);

        cv::waitKey(100);

        if(ui->checkBoxSalIV->isChecked()) cv::imwrite("saida.png",img);
    }while (ui->checkBoxContIV->isChecked());



}

void MainWindow::on_vtnCalibIV_clicked()
{
    std::vector<cv::Mat>imgsIV;

capImg:
    cv::Mat img = imageIVison(ui->spinBoxGain->value(),ui->spinBoxGainAn->value(),ui->spinBoxExp->value(),ui->spinBoxOffS->value());

    cv::Mat imgCinza;

    cv::cvtColor(img,imgCinza,CV_BGR2GRAY);

    cv::Mat imgM,imgCM;

    cv::resize(img,imgM,cv::Size(img.cols/ui->spinBoxFator->value(),img.rows/ui->spinBoxFator->value()));
    cv::resize(imgCinza,imgCM,cv::Size(img.cols/ui->spinBoxFator->value(),img.rows/ui->spinBoxFator->value()));

    cv::imshow("Imagem Para Calibração",imgM);
    cv::imshow("Imagem Cinza",imgCM);

    QMessageBox msgBoxImg;
    msgBoxImg.setText("Imagem Adquirida!");
    msgBoxImg.setInformativeText("Aceitar imagem?");
    msgBoxImg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBoxImg.setDefaultButton(QMessageBox::No);
    int respImg = msgBoxImg.exec();

    if(respImg == QMessageBox::Yes){
        imgsIV.push_back(img);
    }

    if(respImg == QMessageBox::No){
        goto capImg;
    }

    QMessageBox msgBoxCal;
    msgBoxCal.setText("Imagem Salva!");
    msgBoxCal.setInformativeText("Calibrar?");
    msgBoxCal.setStandardButtons(QMessageBox::Yes | QMessageBox::No|QMessageBox::Abort);
    msgBoxCal.setDefaultButton(QMessageBox::No);
    int respCal = msgBoxCal.exec();

    if(respCal == QMessageBox::Yes){
        CalibraCam calibrador;
        calibrador.Calibrar(cv::Size(6,9),2.4,imgsIV.data(),imgsIV.size());
    }

     if(respCal == QMessageBox::No)   goto capImg;

      QMessageBox msgBoxFim;
      msgBoxFim.setText("FIM!");
      msgBoxFim.setInformativeText("Processo Terminado!");
      msgBoxFim.setStandardButtons(QMessageBox::Ok);
      msgBoxFim.exec();


}

void MainWindow::on_horizontalSlider_H_valueChanged(int value)
{
    ui->horizontalSlider_H->setToolTip(QString::number(ui->horizontalSlider_H->value()));
    ui->label_H_Value->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_S_valueChanged(int value)
{
    ui->horizontalSlider_S->setToolTip(QString::number(ui->horizontalSlider_S->value()));
    ui->label_S_Value->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_V_valueChanged(int value)
{
    ui->horizontalSlider_V->setToolTip(QString::number(ui->horizontalSlider_V->value()));
    ui->label_V_Value->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_H_f_valueChanged(int value)
{
    ui->horizontalSlider_H_f->setToolTip(QString::number(ui->horizontalSlider_H_f->value()));
    ui->label_H_Value_f->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_S_f_valueChanged(int value)
{
    ui->horizontalSlider_S_f->setToolTip(QString::number(ui->horizontalSlider_S_f->value()));
    ui->label_S_Value_f->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_V_f_valueChanged(int value)
{
    ui->horizontalSlider_V_f->setToolTip(QString::number(ui->horizontalSlider_V_f->value()));
    ui->label_V_Value_f->setText(QString::number(value));
}
