#include <VISAGE/conectrobo.hpp>
#include <VISAGE/pugixml.hpp>
#include <opencv2/core/core.hpp>
#include <sys/time.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <cassert>

#define S_PORT	    6008
#define MIN_PORT    1024
#define MAX_PORT    32767
#define TAM_MSG     2048


ConectRobo::ConectRobo()
{
    mPorta = S_PORT;
    CriarConexao();
    IniciarLeitura();
}

ConectRobo::ConectRobo(int porta)
{         
    if(porta<=MIN_PORT || porta>MAX_PORT)
    {
        mPorta = S_PORT;
    }
    else
    {
        mPorta = porta;
    }
    CriarConexao();
    IniciarLeitura();
}

void ConectRobo::CriarConexao()
{
    memset((char *) &cliaddr, 0, sizeof(cliaddr));

    sockfd=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
    servaddr.sin_port=htons(mPorta);
    int k = bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr));
    assert(k == 0);
    connect(sockfd,(struct sockaddr *)&cliaddr,sizeof cliaddr);
}

void* ConectRobo::LerMsg(void)
{    
    while (true)
    {
        char req[TAM_MSG];
        len = sizeof(cliaddr);
        n = recvfrom(sockfd,req,TAM_MSG,0,(struct sockaddr *)&cliaddr,&len);
        if(n>-1)
        {
            auto fimMsg = strlen(req);

            char RSI[fimMsg];
            strncpy(RSI, req, fimMsg);
            //      std::cout << "RSI: " <<RSI<<std::endl;
            pugi::xml_document doc;
            pugi::xml_parse_result resultado = doc.load(RSI);

            if (resultado)
            {
                //        std::cout << "RSI recebido:" << std::endl << RSI << std::endl;
                auto ipoc = doc.child("Rob").child("IPOC").text().get();
                //        std::cout << "IPOC recebido: " << ipoc << std::endl;
                mutexIPOC.lock();
                    mFilaIPOC.push(ipoc);
                mutexIPOC.unlock();
                double x = doc.child("Rob").child("RSol").attribute("X").as_double();
                double y = doc.child("Rob").child("RSol").attribute("Y").as_double();
                double z = doc.child("Rob").child("RSol").attribute("Z").as_double();
                double a = doc.child("Rob").child("RSol").attribute("A").as_double();
                double b = doc.child("Rob").child("RSol").attribute("B").as_double();
                double c = doc.child("Rob").child("RSol").attribute("C").as_double();
                mutexInfoRoboRecebe.lock();
                    infoRoboRecebe = InfoRobo(x, y, z, a, b, c);
                mutexInfoRoboRecebe.unlock();
            }
            else
            {
                //          std::cout << "RSI [" << RSI << "] com erro: " << resultado.description() << std::endl;
                std::string ipoc;
                for (int i = 24; i < 34; ++i)
                    ipoc += RSI[i];
                //          std::cout << "IPOC recebido: " << ipoc << std::endl;
                mutexIPOC.lock();
                    mFilaIPOC.push(ipoc);
                mutexIPOC.unlock();
            }

            if (filaInfoRoboEnvia.empty())
                RSI_XML();
            else
            {
                mutexInfoRoboEnvia.lock();
                    auto infoRobo = filaInfoRoboEnvia.front();
                    filaInfoRoboEnvia.pop();
                mutexInfoRoboEnvia.unlock();
                RSI_XML(infoRobo.x, infoRobo.y, infoRobo.z, infoRobo.a, infoRobo.b, infoRobo.c);
            }
        }
    }
}

void ConectRobo::IniciarLeitura()
{
    pthread_t tid;
    int result;
    result = pthread_create(&tid, 0, ConectRobo::chamarLerMsg, this);
    if (result == 0)
        pthread_detach(tid);
}

void ConectRobo::RSI_XML(float x, float y, float z, float a, float b, float c)
{
    std::string IPOC;
    if (!mFilaIPOC.empty())
    {
        std::string RSI("<Sen Type=\"ImFree\"><EStr>ERX Message! Free config!</EStr>");
        RSI += "<RKorr X=\"";
        RSI += std::to_string(x).substr(0, 6);
        RSI += "\" Y=\"";
        RSI += std::to_string(y).substr(0, 6);
        RSI += "\" Z=\"";
        RSI += std::to_string(z).substr(0, 6);
        RSI += "\" A=\"";
        RSI += std::to_string(a).substr(0, 6);
        RSI += "\" B=\"";
        RSI += std::to_string(b).substr(0, 6);
        RSI += "\" C=\"";
        RSI += std::to_string(c).substr(0, 6);
        RSI += "\"/><AKorr A1=\"0.0000\" A2=\"0.0000\" A3=\"0.0000\" A4=\"0.0000\" A5=\"0.0000\" A6=\"0.0000\"/>";
        RSI += "<EKorr E1=\"0.0000\" E2=\"0.0000\" E3=\"0.0000\" E4=\"0.0000\" E5=\"0.0000\" E6=\"0.0000\"/>";
        RSI += "<Tech T21=\"1.09\" T22=\"2.08\" T23=\"3.07\" T24=\"4.06\" T25=\"5.05\" T26=\"6.04\" T27=\"7.03\" T28=\"8.02\" T29=\"9.01\" T210=\"10.00\"/>";
        RSI += "<DiO>0</DiO><IPOC>";
        mutexIPOC.lock();
            IPOC = mFilaIPOC.front();
            RSI += IPOC;
            mFilaIPOC.pop();
        mutexIPOC.unlock();
        RSI += "</IPOC></Sen>";
        pugi::xml_document doc;
        pugi::xml_parse_result resultado = doc.load(RSI.c_str());
        if (resultado)
        {
            sendto(sockfd, RSI.c_str(), strlen(RSI.c_str()), 0, (struct sockaddr *)&cliaddr, len);
            //      std::cout << "  IPOC enviado: " << IPOC << std::endl;
            std::cout << "  Resposta RSI: " << std::endl << RSI << std::endl;
        }
        else
            std::cout << "  Resposta RSI [" << std::endl << RSI << std::endl << "] com erro: " << resultado.description() << std::endl;
    }
}

// Variável Global
ConectRobo conectRobo;
