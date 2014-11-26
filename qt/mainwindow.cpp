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
#include <vector>

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


unsigned int l = 9, a = 6, p = 6008, r = 99;
std::vector<std::string> c = { "169.254.8.106" };
float t = 25.4f;
//cv::Scalar corI[4] =    {cv::Scalar(30,20,90),   cv::Scalar(0,20,90),   cv::Scalar(40,20,90),  cv::Scalar(45,10,20)};
//cv::Scalar corF[4] =    {cv::Scalar(40,250,250), cv::Scalar(20,250,250), cv::Scalar(50,250,250), cv::Scalar(80,250,250)};

//cv::Scalar corI[4] =    {cv::Scalar(20,20,90),   cv::Scalar(0,20,90),   cv::Scalar(30,20,90),  cv::Scalar(45,10,20)};
//cv::Scalar corF[4] =    {cv::Scalar(30,250,250), cv::Scalar(20,250,250), cv::Scalar(45,250,250), cv::Scalar(80,250,250)};

//cv::Scalar corI[4] =    {cv::Scalar(20,20,90),   cv::Scalar(0,20,90),   cv::Scalar(35,20,90),  cv::Scalar(45,10,20)};
//cv::Scalar corF[4] =    {cv::Scalar(35,250,250), cv::Scalar(20,250,250), cv::Scalar(45,250,250), cv::Scalar(80,250,250)};

cv::Scalar corI[4] =    {cv::Scalar(12,20,90),   cv::Scalar(0,20,90),   cv::Scalar(25,20,90),  cv::Scalar(33,10,30)};
cv::Scalar corF[4] =    {cv::Scalar(28,250,250), cv::Scalar(20,250,250), cv::Scalar(45,250,250), cv::Scalar(80,250,250)};

double vMatRC[4][4] = {{1 ,0 ,0, 2600 },
                       {0 ,0 ,1,-3050 },
                       {0 ,-1,0, 1185 },
                       {0 ,0 ,0, 1    }};

double vMarRotRC[3][3] = {{1 ,0 ,0},
                          {0 ,0 ,1},
                          {0 ,-1,0}};

cv::Mat pt;

Programa prog(l, a, t, c, p);
StereoCameras stereoCameras;
bool pararCap = false;

void uso()
{
    std::cerr << "-u   :  [U]so, imprime esta mensagem" << std::endl;
    std::cerr << "-l   :  [L]argura do tabuleiro (L >= 2)" << std::endl;
    std::cerr << "-a   :  [A]ltura do tabuleiro (A >= 2 & A != L)" << std::endl;
    std::cerr << "-t   :  [T]amanho do quadrado em milimetros (T >= 20.0)" << std::endl;
    std::cerr << "-c   :  [C]âmera a ser utilizada (169.254.8.106)" << std::endl;
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
        c = { "169.254.8.106" };
        p = 6008u;
        r = 99u;
        for(int i = 0;i<4;++i){ prog.mMensurium.setarCores(corI[i],corF[i],i);}
    }
    else if ((l < 2u) || (a < 2u || a == l) || (t < 20.f) || (c.empty()) || (p < 1024u || p > 32767u) || (r < 1u || r > 99u))
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

    //cv::Mat_<cv::Vec3b> img= imageIVison(ui->spinBoxGain->value(),ui->spinBoxGainAn->value(),ui->spinBoxExp->value(),ui->spinBoxOffS->value());

    cv::Mat pc = cv::Mat(4,1,CV_64F);
    pc.at<double>(0,0) = pt.at<double>(0,0);
    pc.at<double>(1,0) = pt.at<double>(1,0);
    pc.at<double>(2,0) = pt.at<double>(2,0);
    pc.at<double>(3,0) = 1.f;

    std::cout<<"Ponto Câmera: "<<pc<<std::endl;

    cv::Mat matRC = cv::Mat(4,4,CV_64F,vMatRC);

    cv::Mat pr = matRC*pc;

    std::cout<<"Ponto Robô: "<<pr<<std::endl;

    cv::Mat rotCam = prog.mMensurium.placa[0].marco[0].getOrientacao();

    std::cout<<"Roração Câmera: "<<(180/CV_PI)*rotCam<<std::endl;

    cv::Mat matRotRC = cv::Mat(3,3,CV_64F,vMarRotRC);
    cv::Mat rotR = matRotRC*rotCam;

    std::cout<<"Roração Robô: "<<(180/CV_PI)*rotR<<std::endl;

    //bool resp = true;
    //while(resp){
    //    resp = prog.PosPixel(true);
    //    std::cout<<"Resp= "<<resp<<std::endl;
    //    prog.mMensurium.Rodar("PosPix",img);
    //}

    std::cout<<"FIM PROCESSO!!"<<std::endl;

    //prog.PosIV400();

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
    prog.IniciarCaptura();  
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
    cv::Mat imageToShow;
    do{
        imageToShow  = imageIVison(ui->spinBoxGain->value(),ui->spinBoxGainAn->value(),ui->spinBoxExp->value(),ui->spinBoxOffS->value());

        if (!imageToShow.empty())
        {


            cv::Mat imgPB;
            cv::Mat pbHSV;
            cv::cvtColor(imageToShow,imgPB,CV_BGR2GRAY);

            cv::Mat imgSaida;
            imageToShow.copyTo(imgSaida);


            prog.mMensurium.Rodar("SaidaMe",imgSaida);

            //cv::Mat ptC = prog.mMensurium.placa[0].marco[0].getPosicaoMONO();
            //ptC.copyTo(pt);

            //std::cout << "Ponto: "<< pt << std::endl;


            cv::waitKey(50);
        }
        else
        {
            std::cout << "Imagem vazia." << std::endl;
        }
        cv::waitKey(50);
    }while (ui->checkBoxContIV->isChecked());

}

void MainWindow::on_btnIV_2_clicked()
{
    cv::Mat_<cv::Vec3b> imageToShow;
    bool qrFromCamera = false;
    if (qrFromCamera)
    {
        imageToShow  = imageIVison(ui->spinBoxGain->value(),ui->spinBoxGainAn->value(),ui->spinBoxExp->value(),ui->spinBoxOffS->value());
    }
    else

        if (!imageToShow.empty())
        {


            //            //Se possuir interface grafica
            //            cv::resize(imgSaida,imgSaida,cv::Size(imgSaida.cols/4,imgSaida.rows/4));
            //            cv::namedWindow("imgSaida");
            //            cv::imshow("imgSaida", imgSaida);
            //            if(ui->checkBoxSalIV->isChecked()) cv::imwrite("saida.jpg",imageToShow);

            //            zbar::ImageScanner scanner;
            //            scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

            //            cv::Mat grey;
            //            cv::cvtColor(imageToShow,grey,CV_BGR2GRAY);
            //            int width = imageToShow.cols;
            //            int height = imageToShow.rows;
            //            uchar *raw = (uchar *)grey.data;
            //             //wrap image data
            //            zbar::Image image(width, height, "Y800", raw, width * height);
            //             //scan the image for barcodes
            //            int n = scanner.scan(image);

            //            // extract results
            //            if (n != 0)
            //            {
            //                for(zbar::Image::SymbolIterator symbol = image.symbol_begin();
            //                symbol != image.symbol_end();
            //                ++symbol)
            //                {
            //                    std::vector<cv::Point> vp;
            //                // do something useful with results
            //                std::cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' <<" "<< std::endl;
            //                  int n = symbol->get_location_size();
            //                  for(int i=0;i<n;i++)
            //                  {
            //                    vp.push_back(cv::Point(symbol->get_location_x(i),symbol->get_location_y(i)));
            //                  }
            //                  cv::RotatedRect r = cv::minAreaRect(vp);
            //                         cv::Point2f pts[4];
            //                         r.points(pts);
            //                         for(int i=0;i<4;i++){
            //                           cv::line(imageToShow,pts[i],pts[(i+1)%4],cv::Scalar(255,0,0),3);
            //                         }
            //                         //cout<<"Angle: "<<r.angle<<endl;
            //                 }
            //            }
            //            else std::cout << "Nada encontrado" << std::endl;


            cv::waitKey(50);
        }
        else
        {
            std::cout << "Imagem vazia." << std::endl;

            if (!qrFromCamera)
            {
                zbar::ImageScanner scanner;
                scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

                cv::Mat grey;
                cv::Mat qr = cv::imread("/home/lam/Pictures/qr7.jpg");
                cv::cvtColor(qr,grey,CV_BGR2GRAY);
                int width = qr.cols;
                int height = qr.rows;
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
            }



            cv::waitKey(50);
        }


    std::cout << "IVISION: Finalizando Gigabit DevComm Test" << std::endl;
}

void MainWindow::liberarRecursos()
{
    std::cout << "Liberando recursos..." << std::endl;
    pararCap = true;
    stereoCameras.stop();

    auto cap = prog.getCamera();
    if (cap->isGrabbing())
        cap->stop();
}

cv::Mat img20MP;

void MainWindow::on_btnTesteCor_clicked()
{

    pararCap = false;

    //while(!pararCap){

    QTime t;
    t.start();
    cv::Mat img;
    if(ui->checkBoxVivo->isChecked()){ img =imageIVison(ui->spinBoxGain->value(),ui->spinBoxGainAn->value(),ui->spinBoxExp->value(),ui->spinBoxOffS->value());img20MP = img;}else{img = img20MP;} // If "Vivo" is checked, work with freshly captured image, otherwise work from previous capture
    std::cout<<"Tempo de aquisição: "<<t.restart()<<std::endl;

    if(!img.empty()){

        //                    cv::Mat bgr_image = img;
        //                    cv::Mat lab_image;
        //                    cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

        //                    // Extract the L channel
        //                    std::vector<cv::Mat> lab_planes(3);
        //                    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

        //                    // apply the CLAHE algorithm to the L channel
        //                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        //                    clahe->setClipLimit(4);
        //                    cv::Mat dst;
        //                    clahe->apply(lab_planes[0], dst);

        //                    // Merge the the color planes back into an Lab image
        //                    dst.copyTo(lab_planes[0]);
        //                    cv::merge(lab_planes, lab_image);

        //                   // convert back to RGB
        //                   cv::Mat image_clahe;
        //                   cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);


        std::vector<cv::Mat> channels;
        cv::Mat img_hist_equalized;

        cv::cvtColor(img, img_hist_equalized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format

        cv::split(img_hist_equalized,channels); //split the image into channels

        cv::equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)

        cv::merge(channels,img_hist_equalized); //merge 3 channels including the modified 1st channel into one image

        cv::cvtColor(img_hist_equalized, img_hist_equalized, CV_YCrCb2BGR); //change the color image from YCrCb to BGR format (to display image properly)

        //create windows
        cv::namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
        cv::namedWindow("Histogram Equalized", CV_WINDOW_AUTOSIZE);

        cv::resize(img_hist_equalized,img_hist_equalized,cv::Size(img_hist_equalized.cols/4,img_hist_equalized.rows/4));
        cv::imshow("Histogram Equalized", img_hist_equalized);

        cv::waitKey(0); //wait for key press




        cv::Mat matHSV(img.rows,img.cols,img.type());
        cv::cvtColor(img,matHSV,CV_BGR2HSV); //Image conversion from GBR to HSV space
        //image_clahe.copyTo(matHSV);

        cv::Scalar corIl(ui->horizontalSlider_H->value()  ,ui->horizontalSlider_S->value()  ,ui->horizontalSlider_V->value()); // Lower color bounds acquired from sliders
        cv::Scalar corFl(ui->horizontalSlider_H_f->value(),ui->horizontalSlider_S_f->value(),ui->horizontalSlider_V_f->value()); // Upper color bounds acquired from sliders

        cv::Mat matTh(img.rows,img.cols,CV_8UC1); // Container for output image
        cv::inRange(matHSV,corIl,corFl,matTh); // Output image generated for ranges given


        cv::resize(img,img,cv::Size(img.cols/4,img.rows/4)); //Image is resized to permitted dimensions
        cv::resize(matTh,matTh,cv::Size(matTh.cols/4,matTh.rows/4));
        //cv::resize(image_clahe,image_clahe,cv::Size(image_clahe.cols/4,image_clahe.rows/4));

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
       // cv::imshow("imgCorP",image_clahe);
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
    int nImg = 0;

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
    std::string nomeImg = "img";
    std::string ter = ".png";
    nomeImg = nomeImg+std::to_string(nImg)+ter;
    cv::imwrite(nomeImg,img);
    nImg++;
    std::cout<<nomeImg<<std::endl;
    msgBoxCal.setInformativeText("Calibrar?");
    msgBoxCal.setStandardButtons(QMessageBox::Yes | QMessageBox::No|QMessageBox::Abort);
    msgBoxCal.setDefaultButton(QMessageBox::No);
    int respCal = msgBoxCal.exec();

    if(respCal == QMessageBox::Yes){
        CalibraCam calibrador;
        calibrador.Calibrar(cv::Size(6,9),2.4,imgsIV.data(),imgsIV.size(),"Cal.yml");
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
