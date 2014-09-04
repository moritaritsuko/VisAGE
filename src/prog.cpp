#include <VISAGE/prog.hpp>
#include <VISAGE/conectrobo.hpp>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <cassert>
#include <iostream>


Programa::Programa(unsigned int l, unsigned int a, float t, unsigned int c, unsigned int p)
    : mMensurium()
    , largura(l)
    , altura(a)
    , tamanho(t)
    , camera(c)
    , controleGAMAG(0)
    , mAproximando(false)
{    
    std::cout << "Largura: " << l << std::endl;
    std::cout << "Altura: " << a << std::endl;
    std::cout << "Tamanho (mm): " << t << std::endl;
    std::cout << "Camera: " << c << std::endl;
    std::cout << "Porta RSI: " << p << std::endl;

    //IniciarCaptura();
}



const double setPX = 21.f;//-80.f;//-79.f;
const double setPY = -5.f;//235.f;//56.f;
const double setPZ = 1092.f;
double setPZapx = setPZ;
bool aproxZ = false;
//int tecla;

void Programa::executar(cv::Mat &imgR)
{    
    cv::Mat img;
    if (!filaImagens.empty())
    {
        mutexImagem.lock();
        img = filaImagens.front();
        filaImagens.pop();
        mutexImagem.unlock();
    }

    ConectRobo::InfoRobo infoRobo;
    if (conectRobo.infoRoboRecebe.valido)
    {
        infoRobo = conectRobo.infoRoboRecebe;
        cv::putText(img, cv::format("RSolXYZ(%f, %f, %f)", infoRobo.x,infoRobo.y,infoRobo.z), cv::Point(10, 450), 1, 1, cv::Scalar(255,0,255));
        cv::putText(img, cv::format("RSolABC(%f, %f, %f)", infoRobo.a,infoRobo.b,infoRobo.c), cv::Point(10, 465), 1, 1, cv::Scalar(255,0,255));
    }

    if (!filaMarcadores.empty())
    {
        Marcador marco;
        mutexMarcador.lock();
        marco = filaMarcadores.front();
        filaMarcadores.pop();
        mutexMarcador.unlock();
        auto posicao = marco.getPosicaoMONO();
        auto orientacao = marco.getOrientacao();
        bool orientacaoOK = false;

        auto x = posicao.at<double>(0, 0);
        auto y = posicao.at<double>(1, 0);
        auto z = posicao.at<double>(2, 0);
        auto a = orientacao.at<double>(0, 0);
        auto b = orientacao.at<double>(1, 0);
        auto c = orientacao.at<double>(2, 0);
        double deltaX = setPX - x;
        double deltaY = setPY - y;
        double deltaZ = setPZ - z;
        double xRSI, yRSI, zRSI,aRSI, bRSI, cRSI;
        xRSI = yRSI = zRSI = aRSI = bRSI = cRSI = 0.f;

        cv::putText(img, cv::format("Pos(%f, %f, %f)",x,y,z), cv::Point(10, 105), 1, 1, cv::Scalar(255,0,255));
        cv::putText(img, cv::format("Ori(%f, %f, %f)",(a*180.f)/CV_PI,(b*180.f)/CV_PI,(c*180.f)/CV_PI), cv::Point(10, 120), 1, 1, cv::Scalar(255,0,255));

        if(!orientacaoOK){
            if(c < -0.5f || c > 0.5f){
                aRSI = -0.1f;
                if (c < 0) aRSI = 0.1f;
            }
        }else{

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

        if (xRSI == 0.f && yRSI == 0.f && zRSI == 0.f)
        {
            mAproximando = true;
            char tecla = cv::waitKey(1000);
            MoverPara(0.f,0.f,-z+650);
            cv::waitKey(5000);
            MoverPara(0,0,500);
            cv::waitKey(1000);
            MoverPara(200,200,0);
            cv::waitKey(1000);
            Rotacionar(0,0,90);

        }
    }
        if (!mAproximando)
        {
            conectRobo.mutexInfoRoboEnvia.lock();
            conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(xRSI, yRSI, zRSI, aRSI, 0.f, 0.f, controleGAMAG);
            conectRobo.mutexInfoRoboEnvia.unlock();
        }
        cv::putText(img, cv::format("Delta X: %f", deltaX), cv::Point(10, 15), 1, 1, cv::Scalar(255,0,255));
        cv::putText(img, cv::format("Delta Y: %f", deltaY), cv::Point(10, 30), 1, 1, cv::Scalar(255,0,255));
        cv::putText(img, cv::format("Delta Z: %f", deltaZ), cv::Point(10, 45), 1, 1, cv::Scalar(255,0,255));
    }

    if (!img.empty()){
        img.copyTo(imgR);
        //              cv::imshow("AcharTab", imgR);
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
    if (!filaImagens.empty())
    {
        mutexImagem.lock();
        img = filaImagens.front();
        filaImagens.pop();
        mutexImagem.unlock();
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
        aRSI = 1.f;
        break;

    case 'f':
        aRSI = -1.f;
        break;

    case 't':
        bRSI = 0.1f;
        break;

    case 'g':
        bRSI = -0.1f;
        break;

    case 'y':
        cRSI = 0.1f;
        break;

    case 'h':
        cRSI = -0.1f;
        break;
    }

    conectRobo.mutexInfoRoboEnvia.lock();
    conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(xRSI, yRSI, zRSI,aRSI,bRSI,cRSI,controleGAMAG);
    conectRobo.mutexInfoRoboEnvia.unlock();
    //conectRobo.RSI_XML(xRSI, yRSI, zRSI,aRSI,bRSI,cRSI);

    if (!img.empty())
        cv::imshow("img",img);
    pthread_yield();
}

void Programa::MoverPara(double deltax, double deltay, double deltaz, double vel){

    double xRSI, yRSI, zRSI;
    xRSI = yRSI = zRSI = 0.f;
    ConectRobo::InfoRobo infoRobo;
    if (conectRobo.infoRoboRecebe.valido)
    {
        infoRobo = conectRobo.infoRoboRecebe;

        double pontoInicalX = infoRobo.x+deltax;
        double pontoInicialY = infoRobo.y+deltay;
        double pontoInicialZ = infoRobo.z+deltaz;

        double pontoFinalX = pontoInicalX+deltax;
        double pontoFinalY = pontoInicialY+deltay;
        double pontoFinalZ = pontoInicialZ+deltaz;

        std::cout << "RSOL: " << conectRobo.infoRoboRecebe.x << " " << conectRobo.infoRoboRecebe.y << " " << conectRobo.infoRoboRecebe.z << std::endl;

        if (deltax != 0)
        {
            while(abs(pontoFinalX-infoRobo.x)> 0.1f){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalX= " <<abs(pontoFinalX-infoRobo.x)<<std::endl;
                std::cout << "X= " <<infoRobo.x<<std::endl;
                xRSI = -vel;
                if ((pontoFinalX < pontoInicalX))xRSI = vel;
                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                pthread_yield();
            }
        }
        xRSI = 0.0f;

        if (deltay != 0)
        {
            while(abs(pontoFinalY-infoRobo.y)> 0.1f){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalY= " <<abs(pontoFinalY-infoRobo.y)<<std::endl;
                std::cout << "Y= " <<infoRobo.y<<std::endl;
                yRSI = vel;
                if ((pontoFinalY < pontoInicialY))yRSI = -vel;
                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                pthread_yield();
            }
        }
        yRSI = 0.0f;

        if (deltaz != 0)
        {
            while(abs(pontoFinalZ-infoRobo.z)> 0.1f){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalZ= " <<abs(pontoFinalZ-infoRobo.z)<<std::endl;
                std::cout << "Z= " <<infoRobo.z<<std::endl;
                zRSI = -vel;
                if ((pontoFinalZ < pontoInicialZ))zRSI = vel;
                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                pthread_yield();
            }
        }
        zRSI = 0.0f;
    }
}

void Programa::Rotacionar(double deltaA, double deltaB, double deltaC, double vel){

    std::cout<<"Rotação"<<std::endl;
    double aRSI, bRSI, cRSI;
    aRSI = bRSI = cRSI = 0.f;
    ConectRobo::InfoRobo infoRobo;
    bool deltaValido = false;
    if(deltaA <= 360 && deltaA >= -360) deltaValido = true; else deltaValido = false;
    if(deltaB <= 360 && deltaB >= -360 && deltaValido) deltaValido = true; else deltaValido = false;
    if(deltaC <= 360 && deltaC >= -360 && deltaValido) deltaValido = true; else deltaValido = false;


    if (conectRobo.infoRoboRecebe.valido && deltaValido)
    {
        infoRobo = conectRobo.infoRoboRecebe;

        double pontoFinalA = infoRobo.a+deltaA;

        if((infoRobo.a<0 && deltaA>0)||(infoRobo.a<0 && deltaA<0)){
             pontoFinalA = infoRobo.a - deltaA;
        }

        if(pontoFinalA < -180){
            pontoFinalA = -(pontoFinalA + 180);
        }

        if(pontoFinalA > 180){
            pontoFinalA = -(180 -(pontoFinalA - 180));
        }

         std::cout<<"pontoFinalA= "<<pontoFinalA<<std::endl;

        double pontoFinalB = infoRobo.b+deltaB;

        if((infoRobo.b<0 && deltaB>0)||(infoRobo.b<0 && deltaB<0)){
             pontoFinalB = infoRobo.b - deltaB;
        }

        if(pontoFinalB < -180){
            pontoFinalB = -(pontoFinalB + 180);
        }

        if(pontoFinalB > 180){
            pontoFinalB = -(180 -(pontoFinalB - 180));
        }

        std::cout<<"pontoFinalB= "<<pontoFinalB<<std::endl;

        double pontoFinalC = infoRobo.c+deltaC;

        if((infoRobo.c<0 && deltaC>0)||(infoRobo.c<0 && deltaC<0)){
             pontoFinalC = infoRobo.c - deltaC;
        }

        if(pontoFinalC < -180){
            pontoFinalC = -(pontoFinalC + 180);
        }

        if(pontoFinalC > 180){
            pontoFinalC = -(180 -(pontoFinalC - 180));
        }
        std::cout<<"pontoFinalC= "<<pontoFinalC<<std::endl;

        if (deltaA != 0)
        {
            while(abs(pontoFinalA-infoRobo.a)> 0.1f){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalA= " <<abs(pontoFinalA-infoRobo.a)<<std::endl;
                std::cout << "A= " <<infoRobo.a<<std::endl;
                aRSI = vel;
                if (deltaA > 0)aRSI = -vel;
                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(0.f,0.f,0.f,aRSI, bRSI, cRSI, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
            }
        }
        aRSI = 0.0f;

        if (deltaB != 0)
        {
            while(abs(pontoFinalB-infoRobo.b)> 0.1f){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalB= " <<abs(pontoFinalB-infoRobo.b)<<std::endl;
                std::cout << "B= " <<infoRobo.b<<std::endl;
                bRSI = vel;
                if (deltaB > 0)bRSI = -vel;
                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(0.f,0.f,0.f,aRSI, bRSI, cRSI, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
            }
        }
        bRSI = 0.0f;

        if (deltaC != 0)
        {
            while(abs(pontoFinalC-infoRobo.c)> 0.1f){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalC= " <<abs(pontoFinalC-infoRobo.c)<<std::endl;
                std::cout << "C= " <<infoRobo.c<<std::endl;
                cRSI = -vel;
                if (deltaC > 0)cRSI = vel;
                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(0.f,0.f,0.f,aRSI, bRSI, cRSI, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
            }
        }
        cRSI = 0.0f;

    }
}

void Programa::GAMAG()
{
    conectRobo.mutexInfoRoboEnvia.lock();
    conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, controleGAMAG);
    conectRobo.mutexInfoRoboEnvia.unlock();
}

void Programa::ativarGAMAG()
{
    controleGAMAG = 0;
    GAMAG();
}

void Programa::desativarGAMAG()
{
    controleGAMAG = 1;
    GAMAG();
}

void Programa::IniciarCaptura()
{
    pthread_t tid;
    int result;
    result = pthread_create(&tid, 0, Programa::chamarCapturarImagem, this);
    if (result == 0)
        pthread_detach(tid);
}

void *Programa::CapturarImagem(void)
{
    cv::VideoCapture cap(camera);
    assert(cap.isOpened());

    while (true)
    {

        cv::Mat imagem;
        cap >> imagem;
        Marcador marco;
        mMensurium.AcharCentro1Tab(imagem, marco, largura, altura, tamanho);
        mutexImagem.lock();
        filaImagens.push(imagem);
        mutexImagem.unlock();

        if (marco.isValido())
        {
            mutexMarcador.lock();
            if (filaMarcadores.size() > 10)
                while (!filaMarcadores.empty())
                    filaMarcadores.pop();
            filaMarcadores.push(marco);
            mutexMarcador.unlock();
        }
    }
}
