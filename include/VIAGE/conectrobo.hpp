#ifndef VIAGE_CONECTROBO_HPP
#define VIAGE_CONECTROBO_HPP

#include <netinet/in.h>


class ConectRobo
{
public:
    ConectRobo(int porta);
    void CriarConexao();
    void EnivarMsg(char* msg);
    void InciarLeitura();
    void TesteXML();

private:
    int sockfd,n;
    struct sockaddr_in servaddr,cliaddr;
    socklen_t len;
    char mesg[1024];
    int mPorta;
    void* LerMsg(void);
    static void*  chamarLerMsg(void *arg){return ((ConectRobo*)arg)->LerMsg();}


};

#endif // VIAGE_CONECTROBO_HPP
