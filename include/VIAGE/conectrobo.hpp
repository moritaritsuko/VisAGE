#ifndef VIAGE_CONECTROBO_HPP
#define VIAGE_CONECTROBO_HPP

#include <netinet/in.h>
#include <queue>
#include <string>


class ConectRobo
{
public:
    ConectRobo(int porta);
    void CriarConexao();
    void EnivarMsg(char* msg);
    void IniciarLeitura();
    void RSI_XML(float x = 0.f, float y = 0.f, float z = 0.f, float a = 0.f, float b = 0.f, float c = 0.f);


private:
    int sockfd,n;
    struct sockaddr_in servaddr,cliaddr;
    socklen_t len;
    char msg[1024];
    int mPorta;
    std::queue<std::string> mFilaIPOC;

    void* LerMsg(void);
    static void*  chamarLerMsg(void *arg){return ((ConectRobo*)arg)->LerMsg();}
};

#endif // VIAGE_CONECTROBO_HPP
