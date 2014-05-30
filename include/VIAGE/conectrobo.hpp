#ifndef VIAGE_CONECTROBO_HPP
#define VIAGE_CONECTROBO_HPP

#include <netinet/in.h>
#include <stack>
#include <mutex>
#include <string>


class ConectRobo
{
public:
    ConectRobo(int porta);
    void CriarConexao();
    //void IniciarLeitura();
    void RSI_XML(float x = 0.f, float y = 0.f, float z = 0.f, float a = 0.f, float b = 0.f, float c = 0.f);

    struct InfoRobo
    {
        InfoRobo(){ valido = false; }
        InfoRobo(double X, double Y, double Z, double A, double B, double C)
        : x(X)
        , y(Y)
        , z(Z)
        , a(A)
        , b(B)
        , c(C)
        { valido = true; }
        double x, y, z, a, b, c;
        bool valido;
    };
    std::stack<InfoRobo> pilhaInfoRobo;
    std::mutex mutexInfoRobo;
    void LerMsg();

private:
    int sockfd,n;
    struct sockaddr_in servaddr,cliaddr;
    socklen_t len;
    char msg[2048];
    int mPorta;
    std::stack<std::string> mPilhaIPOC;
    std::mutex mutexIPOC;

    //static void*  chamarLerMsg(void *arg){return ((ConectRobo*)arg)->LerMsg();}
};

#endif // VIAGE_CONECTROBO_HPP
