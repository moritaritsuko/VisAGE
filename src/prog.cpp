#include <VISAGE/prog.hpp>
#include <VISAGE/conectrobo.hpp>

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
    , cap()
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
        bool orientacaoOK = true;

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

        double pontoInicalX = infoRobo.x;
        double pontoInicialY = infoRobo.y;
        double pontoInicialZ = infoRobo.z;

        double pontoFinalX = pontoInicalX+deltax;
        double pontoFinalY = pontoInicialY+deltay;
        double pontoFinalZ = pontoInicialZ+deltaz;



        std::cout << "RSOL: " << conectRobo.infoRoboRecebe.x << " " << conectRobo.infoRoboRecebe.y << " " << conectRobo.infoRoboRecebe.z << std::endl;

        std::cout<<"Delta X ="<<deltax<<std::endl;
        std::cout<<"Delta Y ="<<deltay<<std::endl;
        std::cout<<"Delta Z ="<<deltaz<<std::endl;

        std::cout<<"Ponto Final X ="<<pontoFinalX<<std::endl;
        std::cout<<"Ponto Final Y ="<<pontoFinalY<<std::endl;
        std::cout<<"Ponto Final Z ="<<pontoFinalZ<<std::endl;

        if (deltax != 0)
        {
            while(fabs(pontoFinalX-infoRobo.x)> 1.1*vel){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalX= " <<fabs(pontoFinalX-infoRobo.x)<<std::endl;
                std::cout << "X= " <<infoRobo.x<<std::endl;
                xRSI = vel;
                if ((pontoFinalX < pontoInicalX))xRSI = -vel;
                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                pthread_yield();
                cv::waitKey(50);
            }
        }
        xRSI = 0.0f;

        if (deltay != 0)
        {
            while(fabs(pontoFinalY-infoRobo.y)> 1.1*vel){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "Yif= " <<pontoInicialY;std::cout << " | Yf= " <<pontoFinalY;std::cout << " | Yat = " <<infoRobo.y;
                std::cout << " | detalY= " <<fabs(pontoFinalY-infoRobo.y);std::cout << " | inc= " <<yRSI<<std::endl;

                yRSI = vel;
                if ((pontoFinalY < pontoInicialY))yRSI = -vel;
                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                pthread_yield();
                cv::waitKey(50);
            }
        }
        yRSI = 0.0f;

        if (deltaz != 0)
        {
            float deltaLz = fabs(pontoFinalZ-infoRobo.z);
            while(deltaLz> 1.1*vel){
                infoRobo = conectRobo.infoRoboRecebe;

                std::cout << "detalZ= " <<deltaLz;
                deltaLz = fabs(pontoFinalZ-infoRobo.z);
                std::cout << " Z= " <<infoRobo.z<<std::endl;

                zRSI = vel;
                if ((pontoFinalZ < pontoInicialZ))zRSI = -vel;

                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(xRSI, yRSI, zRSI, 0.f, 0.f, 0.f, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                pthread_yield();

                cv::waitKey(50);
            }
        }
        zRSI = 0.0f;
    }

    std::cout<<"FIM DO MOVIMENTAÇÃO!"<<std::endl;
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

        std::cout<<"Info robo Ang "<<infoRobo.a<<","<<infoRobo.b<<","<<infoRobo.c<<std::endl;

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

            while(fabs(pontoFinalA-infoRobo.a)> 1.1f*vel){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalA= " <<fabs(pontoFinalA-infoRobo.a)<<std::endl;
                std::cout << "A= " <<infoRobo.a<<std::endl;

                aRSI = vel;
                if ((pontoFinalA - infoRobo.a) < 0)aRSI = -vel;


                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(0.f, 0.f, 0.f, aRSI, 0.f, 0.f, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                pthread_yield();
                cv::waitKey(50);
            }
        }
        aRSI = 0.0f;

        if (deltaB != 0)
        {

            while(fabs(pontoFinalB-infoRobo.b)> 1.1f*vel){
                infoRobo = conectRobo.infoRoboRecebe;


                //if((infoRobo.b>-vel)&&(infoRobo.b<vel)) vel = -vel; else


                std::cout << "Ang B = " <<infoRobo.b;
                std::cout << " | detalB = " <<fabs(pontoFinalB-infoRobo.b);
                std::cout << " | incremento = " <<bRSI<<std::endl;

                bRSI = vel;
                if ((pontoFinalB - infoRobo.b) < 0)bRSI = -vel;

                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(0.f,0.f,0.f,0.f, bRSI, 0.f, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                cv::waitKey(50);
            }
        }
        bRSI = 0.0f;

        if (deltaC != 0)
        {
            cRSI = -vel;
            if (deltaC > 0)cRSI = vel;

            while(fabs(pontoFinalC-infoRobo.c)> 1.1f*vel){
                infoRobo = conectRobo.infoRoboRecebe;
                std::cout << "detalC= " <<fabs(pontoFinalC-infoRobo.c)<<std::endl;
                std::cout << "C= " <<infoRobo.c<<std::endl;

                conectRobo.mutexInfoRoboEnvia.lock();
                conectRobo.infoRoboEnvia = ConectRobo::InfoRobo(0.f,0.f,0.f,0.f, 0.f, cRSI, controleGAMAG);
                conectRobo.mutexInfoRoboEnvia.unlock();
                cv::waitKey(50);
            }
        }
        cRSI = 0.0f;

    }

    std::cout<<"FIM DO ROTAÇÃO!"<<std::endl;
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
    cap = cv::VideoCapture(camera);
    pthread_t tid;
    int result;
    result = pthread_create(&tid, 0, Programa::chamarCapturarImagem, this);
    if (result == 0)
        pthread_detach(tid);
    cv::waitKey(100);
}

void *Programa::CapturarImagem(void)
{
    if (!cap.isOpened())
        cap.open(camera);
    cv::waitKey(30);
    assert(cap.isOpened());
    std::cout << "Camera Mono ativada" << std::endl;
    while (cap.isOpened())
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

void Programa::CapturaCameraMono()
{
    if (!filaImagens.empty())
    {
        mutexImagem.lock();
        cv::Mat img = filaImagens.front();
        filaImagens.pop();
        mutexImagem.unlock();
        cv::waitKey(30);
        if (!img.empty())
            cv::imshow("Camera Mono", img);
    }
}

void Programa::PosIV400(){

    //mMensurium.Rodar("PosIV400",img);
    float* deltaPos = new float[3];
    float* deltaAng = new float[3];
    mMensurium.getDeltaXYZ(deltaPos);
    mMensurium.getDeltaABC(deltaAng);

    std::cout<<"Delta Pos: "<<deltaPos[0]<<" , "<<deltaPos[1]<<" , "<<deltaPos[2]<<std::endl;
    std::cout<<"Delta Ang: "<<deltaAng[0]<<" , "<<deltaAng[1]<<" , "<<deltaAng[2]<<std::endl;
    std::cout<<"****************************************************DELTA ANG ENVIADO : "<<deltaAng[2]<<"********************************************************************"<<std::endl;
    Rotacionar(0.f,deltaAng[2],0.f);
    std::cout<<"****************************************************Processo Concluido Ang!!!!!!!!!!!!"<<"********************************************************************"<<std::endl;

    std::cout<<"****************************************************DELTA POS ENVIADO : "<<deltaPos[2]<<"********************************************************************"<<std::endl;
    MoverPara(0.f,deltaPos[2],deltaPos[0]);
    std::cout<<"****************************************************Processo Concluido Pos!!!!!!!!!!!!"<<"********************************************************************"<<std::endl;



    //MoverPara(deltaPos[0],deltaPos[1],deltaPos[2]);
}

float calcAng(cv::Point2f ptCG, cv::Point2f ptCe){
    double r = sqrt((ptCG.x-ptCe.x)*(ptCG.x-ptCe.x)+(ptCG.y-ptCe.y)*(ptCG.y-ptCe.y));
    std::cout<<"r = "<<r;
    float projSin = (ptCe.y - ptCG.y)/r;
    std::cout<<" projSin = "<<projSin;
    float ang = asin(projSin);
    std::cout<<" asin = "<<ang;
    ang = 180*ang/CV_PI;
    std::cout<<" ang = "<<ang<<std::endl;
    return ang;
}


bool Programa::PosPixel(bool temImg){
    CvPoint2D32f* pD0 = mMensurium.placa[0].marco[0].getCantosDigonal();
    CvPoint2D32f* pD2 = mMensurium.placa[0].marco[2].getCantosDigonal();
    CvPoint2D32f* pD1 = mMensurium.placa[0].marco[1].getCantosDigonal();
    CvPoint2D32f* pD3 = mMensurium.placa[0].marco[3].getCantosDigonal();

    cv::Point2f centro0 = mMensurium.placa[0].marco[0].getCentroImg();
    cv::Point2f centro2 = mMensurium.placa[0].marco[2].getCentroImg();
    cv::Point2f centro1 = mMensurium.placa[0].marco[1].getCentroImg();
    cv::Point2f centro3 = mMensurium.placa[0].marco[3].getCentroImg();

    float angA = calcAng(centro2,centro0);
    float angR = calcAng(centro3,centro1);

    std::cout<<"Anulo In= "<<angA<<std::endl;
    std::cout<<"Anulo Fi= "<<angR<<std::endl;

    //float delta = (angR-angA);

    while(fabs(angR-angA)>0.1){
        if((angR-angA)<0){
            Rotacionar(0.f,0.02,0.f,0.01);
            angA -= 0.02;
        }else{
            Rotacionar(0.f,-0.02,0.f,0.01);
            angA += 0.02;
        }
        std::cout<<"Anulo A= "<<angA<<std::endl;
    }

    float tamM0x = pD0[0].x-pD0[1].x;
    float tamM2x = pD2[0].x-pD2[1].x;

    float tamM0y = pD0[0].y-pD0[1].y;
    float tamM2y = pD2[0].y-pD2[1].y;

    std::cout<<"pD0[0]= "<<pD0[0].x<<" , "<<pD0[0].y<<std::endl;
    std::cout<<"pD0[1]= "<<pD0[1].x<<" , "<<pD0[1].y<<std::endl;
    std::cout<<"tamM0x= "<<tamM0x<<std::endl;
    std::cout<<"tamM0y= "<<tamM0y<<std::endl;
    std::cout<<"pD2[0]= "<<pD2[0].x<<" , "<<pD2[0].y<<std::endl;
    std::cout<<"pD2[1]= "<<pD2[1].x<<" , "<<pD2[1].y<<std::endl;
    std::cout<<"tamM2x= "<<tamM2x<<std::endl;
    std::cout<<"tamM2y= "<<tamM2y<<std::endl;

    std::cout<<"delta C= "<<fabs(tamM0x-tamM2x)<<std::endl;
    std::cout<<"delta A= "<<fabs(tamM0y-tamM2y)<<std::endl;

    bool result = false;
    if(temImg){
        if(fabs(tamM0x-tamM2x)>1.f){
            if((tamM0x-tamM2x) > 0){
                std::cout<<"Enviando Correção C pixel + "<<std::endl;
                Rotacionar(0.f,0.f,-0.6f,0.1);
            }else{
                std::cout<<"Enviando Correção C pixel - "<<std::endl;
                Rotacionar(0.f,0.f,-0.6f,0.1);
            }
            result = true;
        }else{
            result = false;
        }

        if(fabs(tamM0y-tamM2y)>1.f){
            if((tamM0y-tamM2y) > 0){
                std::cout<<"Enviando Correção A pixel + "<<std::endl;
                Rotacionar(-0.6,0.f,0.f,0.1);
            }else{
                std::cout<<"Enviando Correção A pixel - "<<std::endl;
                Rotacionar(0.6,0.f,0.f,0.1);
            }
            result = true;
        }else{
            if(!result)result = false;
        }
    }


return result;

}
