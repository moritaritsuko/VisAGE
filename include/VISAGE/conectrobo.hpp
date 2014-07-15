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
        InfoRobo()
        : x(0.f)
        , y(0.f)
        , z(0.f)
        , a(0.f)
        , b(0.f)
        , c(0.f)
        , mag(0)
        , change(0)
        , vel(0)
        , valido(false)
        { }

        InfoRobo(double X, double Y, double Z, double A, double B, double C)
        : x(X)
        , y(Y)
        , z(Z)
        , a(A)
        , b(B)
        , c(C)
        , mag(0)
        , change(0)
        , vel(0)
        , valido(true)
        { }

        InfoRobo(double X, double Y, double Z, double A, double B, double C, int MAG, int CHANGE, int VEL)
        : x(X)
        , y(Y)
        , z(Z)
        , a(A)
        , b(B)
        , c(C)
        , mag(MAG)
        , change(CHANGE)
        , vel(VEL)
        , valido(true)
        { }

        double x, y, z, a, b, c;
        int mag, change, vel;
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

public:
    void RSI_XML(float x = 0.f, float y = 0.f, float z = 0.f, float a = 0.f, float b = 0.f, float c = 0.f, int mag = 0, int change = 0, int vel = 0);
};

extern ConectRobo conectRobo;

#endif // VISAGE_CONECTROBO_HPP
