#include <VISAGE/prog.hpp>
#include <VISAGE/conectrobo.hpp>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <cassert>
#include <iostream>


Programa::Programa(unsigned int l, unsigned int a, float t, unsigned int c, unsigned int p)
    : mMensurium(l, a, t, c)
{    
    std::cout << "Largura: " << l << std::endl;
    std::cout << "Altura: " << a << std::endl;
    std::cout << "Tamanho (mm): " << t << std::endl;
    std::cout << "Camera: " << c << std::endl;
    std::cout << "Porta RSI: " << p << std::endl;
}



const double setPX = 21.f;//-80.f;//-79.f;
const double setPY = -5.f;//235.f;//56.f;
const double setPZ = 1092.f;
double setPZapx = setPZ;
bool aproxZ = false;
//int tecla;

void Programa::executar(cv::Mat &imgR)
{
    auto tempo = (double) cv::getTickCount();

    cv::Mat img;
    if (!mMensurium.filaImagens.empty())
    {
        mMensurium.mutexImagem.lock();
        img = mMensurium.filaImagens.front();
        mMensurium.filaImagens.pop();
        mMensurium.mutexImagem.unlock();
    }

    ConectRobo::InfoRobo infoRobo;
    if (conectRobo.infoRoboRecebe.valido)
    {
        conectRobo.mutexInfoRoboRecebe.lock();
        infoRobo = conectRobo.infoRoboRecebe;
        conectRobo.mutexInfoRoboRecebe.unlock();
        cv::putText(img, cv::format("RSolXYZ(%f, %f, %f)", infoRobo.x,infoRobo.y,infoRobo.z), cv::Point(10, 450), 1, 1, cv::Scalar(255,0,255));
        cv::putText(img, cv::format("RSolABC(%f, %f, %f)", infoRobo.a,infoRobo.b,infoRobo.c), cv::Point(10, 465), 1, 1, cv::Scalar(255,0,255));
    }

    if (!mMensurium.filaMarcadores.empty())
    {
        Marcador marco;
        mMensurium.mutexMarcador.lock();
        marco = mMensurium.filaMarcadores.front();
        mMensurium.filaMarcadores.pop();
        mMensurium.mutexMarcador.unlock();
        auto posicao = marco.getPosicao();
        auto orientacao = marco.getOrientacao();

        auto x = posicao.at<double>(0, 0);
        auto y = posicao.at<double>(1, 0);
        auto z = posicao.at<double>(2, 0);
        auto a = orientacao.at<double>(0, 0);
        auto b = orientacao.at<double>(1, 0);
        auto c = orientacao.at<double>(2, 0);
        double deltaX = setPX - x;
        double deltaY = setPY - y;
        double deltaZ = setPZ - z;
        double xRSI, yRSI, zRSI;
        xRSI = yRSI = zRSI = 0.f;

        cv::putText(img, cv::format("Pos(%f, %f, %f)",x,y,z), cv::Point(10, 105), 1, 1, cv::Scalar(255,0,255));
        cv::putText(img, cv::format("Ori(%f, %f, %f)",a,b,c), cv::Point(10, 120), 1, 1, cv::Scalar(255,0,255));

        if(deltaZ < -10.f || deltaZ > 10.f)
        {
            if(deltaZ > 0.f)
            {
                cv::putText(img, cv::format("DirZ: <"), cv::Point(10, 90), 1, 1, cv::Scalar(255,0,255));
                zRSI = -1.0f;
            }
            else if(deltaZ < 0.f)
            {
                cv::putText(img, cv::format("DirZ: >"), cv::Point(10, 90), 1, 1, cv::Scalar(255,0,255));
                zRSI = 1.0f;
            }
        }
        else
        {
            cv::putText(img, cv::format("DirZ: CENTRO"), cv::Point(10, 90), 1, 1, cv::Scalar(255,0,255));

            if(deltaY < -2.5f || deltaY > 2.5f)
            {
                if(deltaY > 0.f)
                {
                    cv::putText(img, cv::format("DirY: <"), cv::Point(10, 75), 1, 1, cv::Scalar(255,0,255));
                    xRSI = -1.0f;
                }
                else if(deltaY < 0.f)
                {
                    cv::putText(img, cv::format("DirY: >"), cv::Point(10, 75), 1, 1, cv::Scalar(255,0,255));
                    xRSI = 1.0f;
                }
            }
            else
            {
                cv::putText(img, cv::format("DirY: CENTRO"), cv::Point(10, 75), 1, 1, cv::Scalar(255,0,255));
                xRSI = 0.f;
            }

            if(deltaX < -2.5f || deltaX > 2.5f)
            {
                if(deltaX > 0.f)
                {
                    cv::putText(img, cv::format("DirX: <"), cv::Point(10, 60), 1, 1, cv::Scalar(255,0,255));
                    yRSI = 1.0f;
                }
                else if(deltaX < 0.f)
                {
                    cv::putText(img, cv::format("DirX: >"), cv::Point(10, 60), 1, 1, cv::Scalar(255,0,255));
                    yRSI = -1.0f;
                }
            }
            else
            {
                cv::putText(img, cv::format("DirX: CENTRO"), cv::Point(10, 60), 1, 1, cv::Scalar(255,0,255));
                yRSI = 0.f;
            }
        }

        if (xRSI == 0.f && yRSI == 0.f && zRSI == 0.f && infoRobo.valido)
        {
            char tecla = cv::waitKey(3);
            if (tecla == 'p' || tecla == 'P' || !aproxZ)
            {
                MoverPara(0.f,0.f,z);
            }
        }

        conectRobo.filaInfoRoboEnvia.push(ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f));
        cv::putText(img, cv::format("Delta X: %f", deltaX), cv::Point(10, 15), 1, 1, cv::Scalar(255,0,255));
        cv::putText(img, cv::format("Delta Y: %f", deltaY), cv::Point(10, 30), 1, 1, cv::Scalar(255,0,255));
        cv::putText(img, cv::format("Delta Z: %f", deltaZ), cv::Point(10, 45), 1, 1, cv::Scalar(255,0,255));
    }
    tempo = (double) cv::getTickCount() - tempo;
    std::cout << "              Resposta RSI em " << tempo * 1000.f / cv::getTickFrequency() << " ms." << std::endl;

    if (!img.empty()){
        img.copyTo(imgR);
        //            cv::imshow("AcharTab", imgR);
    }else{
        imgR = cv::Mat();
    }
    //    }
}

void Programa::Manipular(){

    double xRSI, yRSI, zRSI;
    double aRSI, bRSI, cRSI;
    xRSI = yRSI = zRSI = 0.f;
    aRSI = bRSI = cRSI = 0.f;

    cv::Mat img;
    if (!mMensurium.filaImagens.empty())
    {
        mMensurium.mutexImagem.lock();
        img = mMensurium.filaImagens.front();
        mMensurium.filaImagens.pop();
        mMensurium.mutexImagem.unlock();
    }

    char tecla = cv::waitKey(3);
    //    std::cout<<"Tecla: "<<(int)tecla<<std::endl;
    switch (tecla){
    case 'w':
        xRSI = 1.f;
        break;

    case 's':
        xRSI = -1.f;
        break;

    case 'a':
        yRSI = 1.f;
        break;

    case 'd':
        yRSI = -1.f;
        break;

    case 'r':
        aRSI = 0.999999f;
        break;

    case 'f':
        aRSI = -0.999999f;
        break;

    case 't':
        bRSI = 0.5f;
        break;

    case 'g':
        bRSI = -0.5f;
        break;

    case 'y':
        cRSI = 0.5f;
        break;

    case 'h':
        cRSI = -0.5f;
        break;
    }

    conectRobo.filaInfoRoboEnvia.push(ConectRobo::InfoRobo(xRSI, yRSI, zRSI,aRSI,bRSI,cRSI));

    if (!img.empty())
        cv::imshow("img",img);


}

void Programa::MoverPara(double deltax, double deltay, double deltaz){

    double xRSI, yRSI, zRSI;
    xRSI = yRSI = zRSI = 0.f;
    ConectRobo::InfoRobo infoRobo;
    if (conectRobo.infoRoboRecebe.valido)
    {
        conectRobo.mutexInfoRoboRecebe.lock();
        infoRobo = conectRobo.infoRoboRecebe;
        conectRobo.mutexInfoRoboRecebe.unlock();

        double pontoFinalX = infoRobo.x+deltax;
        double pontoFinalY = infoRobo.y+deltay;
        double pontoFinalZ = infoRobo.z+deltaz;

        while(abs(pontoFinalX-infoRobo.x)> 0.1f){
            std::cout << "detalX= " <<abs(pontoFinalX-infoRobo.x)<<std::endl;
            std::cout << "X= " <<infoRobo.x<<std::endl;
            xRSI = 1.0f;
            if (deltax > 0)xRSI = -1.0f;
            conectRobo.filaInfoRoboEnvia.push(ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f));
        }
        xRSI = 0.0f;

        while(abs(pontoFinalY-infoRobo.y)> 0.1f){
            conectRobo.mutexInfoRoboRecebe.lock();
            infoRobo = conectRobo.infoRoboRecebe;
            conectRobo.mutexInfoRoboRecebe.unlock();
            std::cout << "detalY= " <<abs(pontoFinalY-infoRobo.y)<<std::endl;
            std::cout << "Y= " <<infoRobo.y<<std::endl;
            yRSI = -1.0f;
            if (deltax > 0)yRSI = 1.0f;
            conectRobo.filaInfoRoboEnvia.push(ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f));
        }
        yRSI = 0.0f;

        while(abs(pontoFinalZ-infoRobo.z)> 0.1f){
            conectRobo.mutexInfoRoboRecebe.lock();
            infoRobo = conectRobo.infoRoboRecebe;
            conectRobo.mutexInfoRoboRecebe.unlock();
            std::cout << "detalZ= " <<abs(pontoFinalZ-infoRobo.z)<<std::endl;
            std::cout << "Z= " <<infoRobo.z<<std::endl;
            zRSI = 1.0f;
            if (deltax > 0)zRSI = -1.0f;
            conectRobo.filaInfoRoboEnvia.push(ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f));
        }
        zRSI = 0.0f;
    }
}

void Programa::Rotacionar(double deltaA, double deltaB, double deltaC){

    double aRSI, bRSI, cRSI;
    aRSI = bRSI = bRSI = 0.f;
    ConectRobo::InfoRobo infoRobo;

    if (conectRobo.infoRoboRecebe.valido)
    {
        conectRobo.mutexInfoRoboRecebe.lock();
        infoRobo = conectRobo.infoRoboRecebe;
        conectRobo.mutexInfoRoboRecebe.unlock();

        double pontoFinalA = infoRobo.a+deltaA;
        double pontoFinalB = infoRobo.b+deltaB;
        double pontoFinalC = infoRobo.c+deltaC;

        while(abs(pontoFinalA-infoRobo.a)> 0.1f){
            conectRobo.mutexInfoRoboRecebe.lock();
            infoRobo = conectRobo.infoRoboRecebe;
            conectRobo.mutexInfoRoboRecebe.unlock();
            std::cout << "detalX= " <<abs(pontoFinalA-infoRobo.a)<<std::endl;
            std::cout << "X= " <<infoRobo.a<<std::endl;
            aRSI = 1.0f;
            if (deltaA > 0)aRSI = -1.0f;
            conectRobo.filaInfoRoboEnvia.push(ConectRobo::InfoRobo(0.f,0.f,0.f,aRSI, bRSI, cRSI));
        }
        aRSI = 0.0f;


        while(abs(pontoFinalB-infoRobo.b)> 0.1f){
            conectRobo.mutexInfoRoboRecebe.lock();
            infoRobo = conectRobo.infoRoboRecebe;
            conectRobo.mutexInfoRoboRecebe.unlock();
            std::cout << "detalY= " <<abs(pontoFinalB-infoRobo.b)<<std::endl;
            std::cout << "Y= " <<infoRobo.b<<std::endl;
            bRSI = -1.0f;
            if (deltaB > 0)bRSI = 1.0f;
            conectRobo.filaInfoRoboEnvia.push(ConectRobo::InfoRobo(0.f,0.f,0.f,aRSI, bRSI, cRSI));
        }
        bRSI = 0.0f;

        while(abs(pontoFinalC-infoRobo.c)> 0.1f){
            conectRobo.mutexInfoRoboRecebe.lock();
            infoRobo = conectRobo.infoRoboRecebe;
            conectRobo.mutexInfoRoboRecebe.unlock();
            std::cout << "detalZ= " <<abs(pontoFinalC-infoRobo.c)<<std::endl;
            std::cout << "Z= " <<infoRobo.c<<std::endl;
            cRSI = 1.0f;
            if (deltaC > 0)cRSI = -1.0f;
            conectRobo.filaInfoRoboEnvia.push(ConectRobo::InfoRobo(0.f,0.f,0.f,aRSI, bRSI, cRSI));
        }
        cRSI = 0.0f;

    }
}

