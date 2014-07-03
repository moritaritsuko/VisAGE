#ifndef VISAGE_CONECTROBO_HPP
#define VISAGE_CONECTROBO_HPP

#include <netinet/in.h>
#include <queue>
#include <mutex>
#include <string>


class ConectRobo
{
public:
    ConectRobo();
    ConectRobo(int porta);
    void CriarConexao();
    void IniciarLeitura();

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

    InfoRobo infoRoboRecebe;
    std::mutex mutexInfoRoboRecebe;
    InfoRobo infoRoboEnvia;
    std::mutex mutexInfoRoboEnvia;

private:
    int sockfd,n;
    struct sockaddr_in servaddr,cliaddr;
    socklen_t len;
    char msg[2048];
    int mPorta;
    std::queue<std::string> mFilaIPOC;
    std::mutex mutexIPOC;

    void* LerMsg(void);
    static void* chamarLerMsg(void *arg){ return ((ConectRobo*)arg)->LerMsg(); }

    void RSI_XML(float x = 0.f, float y = 0.f, float z = 0.f, float a = 0.f, float b = 0.f, float c = 0.f);
};

extern ConectRobo conectRobo;

#endif // VISAGE_CONECTROBO_HPP
