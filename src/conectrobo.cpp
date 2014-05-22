#include <VIAGE/conectrobo.hpp>
#include <VIAGE/pugixml.hpp>
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
#include <iostream>

#define S_PORT	    8001
#define MIN_PORT    1024
#define MAX_PORT    32767
#define TAM_MSG     1024
#define LEN         128


ConectRobo::ConectRobo(int porta)
{         
    if(porta<=MIN_PORT || porta>MAX_PORT)
    {
        mPorta=S_PORT;
    }
    else
    {
        mPorta = porta;
    }
}

void ConectRobo::CriarConexao(){

    memset((char *) &cliaddr, 0, sizeof(cliaddr));

    sockfd=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
    servaddr.sin_port=htons(mPorta);
    int k = bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr));
        //printf("Servidor Iniciado! k= %i\n",k);
        std::cout<<"Servidor Iniciado! k= "<<k<<std::endl;


}

void ConectRobo::EnivarMsg(char *msg){


}

void *ConectRobo::LerMsg(void){
    connect(sockfd,(struct sockaddr *)&cliaddr,sizeof cliaddr);
    for (;;)
    {
       len = sizeof(cliaddr);
       n = recvfrom(sockfd,mesg,512,0,(struct sockaddr *)&cliaddr,&len);
       //printf("n = [%i]\n",n);
       if(n>-1){
           //mesg[n] = 0;
           printf("Recebido[%i]: %s",n,mesg);
           std::cout<<"Recebido = "<<mesg<<std::endl;
       }
     }
}


void ConectRobo::InciarLeitura(){

    pthread_t tid;
     int       result;
     result = pthread_create(&tid, 0, ConectRobo::chamarLerMsg, this);
     if (result == 0)
        pthread_detach(tid);

}

void ConectRobo::TesteXML(){
//    pugi::xml_document doc;

//    //[code_modify_add
//    // add node with some name
//    pugi::xml_node node = doc.append_child("Sen");
//    node.set_value("Type=\"ImFree\"");


//    // add description node with text child
//    pugi::xml_node descr = node.append_child("EStr");
//    descr.append_child(pugi::node_pcdata).set_value("ERX Message! Free config!");

//    // add param node before the description
//    pugi::xml_node param = node.insert_child_after("RKorr", descr);

//    // add attributes to param node
//    param.append_attribute("X") = "0.0000";
//    param.append_attribute("Y") = "0.0000";
//    param.append_attribute("Z") = "0.0000";
//    param.append_attribute("A") = "0.0000";
//    param.append_attribute("B") = "0.0000";
//    param.append_attribute("C") = "0.0000";

//    pugi::xml_node param2 = node.insert_child_after("AKorr", descr);

//    // add attributes to param node
//    param2.append_attribute("A1") = "0.0000";
//    param2.append_attribute("A2") = "0.0000";
//    param2.append_attribute("A3") = "0.0000";
//    param2.append_attribute("A4") = "0.0000";
//    param2.append_attribute("A5") = "0.0000";
//    param2.append_attribute("A6") = "0.0000";

//    //param.append_attribute("value") = 1.1;
//    //param.insert_attribute_after("type", param.attribute("name")) = "float";
//    //]

//    doc.print(std::cout);
//    std::cout << "Saving result: " << doc.save_file("save_file_output.xml") << std::endl;
}


