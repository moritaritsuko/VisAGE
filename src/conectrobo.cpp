#include <VIAGE/conectrobo.hpp>
#include <VIAGE/pugixml.hpp>
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
#include <mutex>

#define S_PORT	    8001
#define MIN_PORT    1024
#define MAX_PORT    32767
#define TAM_MSG     1024
#define LEN         128

std::mutex mutexIPOC;

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
}

void ConectRobo::EnivarMsg(char *msg)
{


}

void *ConectRobo::LerMsg(void)
{    
  connect(sockfd,(struct sockaddr *)&cliaddr,sizeof cliaddr);
  for (;;)
  {
    len = sizeof(cliaddr);
    n = recvfrom(sockfd,msg,512,0,(struct sockaddr *)&cliaddr,&len);
    if(n>-1)
    {
      pugi::xml_document doc;
      pugi::xml_parse_result resultado = doc.load(msg);

      if (resultado)
      {
        auto ipoc = doc.child("IPOC").text().get();
        std::cout << "IPOC recebido: " << ipoc << std::endl;
        mutexIPOC.lock();
          mFilaIPOC.push(ipoc);
        mutexIPOC.unlock();
      }
      else
          std::cout << "RSI [" << msg << "] com erro: " << resultado.description() << std::endl;
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
  if (!mFilaIPOC.empty())
  {
    pugi::xml_document doc;
    auto tempoInicial = cv::getTickCount();
    // <Sen Type="ImFree">
    pugi::xml_node senNode = doc.append_child("Sen");
    senNode.append_attribute("Type") = "ImFree";
      // <EStr>ERX Message! Free config!</EStr>
    pugi::xml_node estrNode = senNode.append_child("EStr");
    estrNode.append_child(pugi::node_pcdata).set_value("ERX Message! Free config!");
      // <RKorr X="0.0000" Y="0.0000" Z="0.0000" A="0.0000" B="0.0000" C="0.0000"/>
    pugi::xml_node rkorrNode = senNode.insert_child_after("RKorr", estrNode);
    rkorrNode.append_attribute("X") = std::to_string(x).substr(0, 6).c_str();
    rkorrNode.append_attribute("Y") = std::to_string(y).substr(0, 6).c_str();
    rkorrNode.append_attribute("Z") = std::to_string(z).substr(0, 6).c_str();
    rkorrNode.append_attribute("A") = std::to_string(a).substr(0, 6).c_str();
    rkorrNode.append_attribute("B") = std::to_string(b).substr(0, 6).c_str();
    rkorrNode.append_attribute("C") = std::to_string(c).substr(0, 6).c_str();
      // <AKorr A1="0.0000" A2="0.0000" A3="0.0000" A4="0.0000" A5="0.0000" A6="0.0000" />
    pugi::xml_node akorrNode = senNode.insert_child_after("AKorr", rkorrNode);
    akorrNode.append_attribute("A1") = "0.0000";
    akorrNode.append_attribute("A2") = "0.0000";
    akorrNode.append_attribute("A3") = "0.0000";
    akorrNode.append_attribute("A4") = "0.0000";
    akorrNode.append_attribute("A5") = "0.0000";
    akorrNode.append_attribute("A6") = "0.0000";
      // <EKorr E1="0.0000" E2="0.0000" E3="0.0000" E4="0.0000" E5="0.0000" E6="0.0000" />
    pugi::xml_node ekorrNode = senNode.insert_child_after("EKorr", akorrNode);
    ekorrNode.append_attribute("E1") = "0.0000";
    ekorrNode.append_attribute("E2") = "0.0000";
    ekorrNode.append_attribute("E3") = "0.0000";
    ekorrNode.append_attribute("E4") = "0.0000";
    ekorrNode.append_attribute("E5") = "0.0000";
    ekorrNode.append_attribute("E6") = "0.0000";
      // <Tech T21="1.09" T22="2.08" T23="3.07" T24="4.06" T25="5.05" T26="6.04" T27="7.03" T28="8.02" T29="9.01" T210="10.00" />
    pugi::xml_node techNode = senNode.insert_child_after("Tech", ekorrNode);
    techNode.append_attribute("T21") = "1.09";
    techNode.append_attribute("T22") = "2.08";
    techNode.append_attribute("T23") = "3.07";
    techNode.append_attribute("T24") = "4.06";
    techNode.append_attribute("T25") = "5.05";
    techNode.append_attribute("T26") = "6.04";
    techNode.append_attribute("T27") = "7.03";
    techNode.append_attribute("T28") = "8.02";
    techNode.append_attribute("T29") = "9.01";
    techNode.append_attribute("T210") = "10.00";
      // <DiO>125</DiO>
    pugi::xml_node dioNode = senNode.insert_child_after("DiO", techNode);
    dioNode.append_child(pugi::node_pcdata).set_value("125");
      // <IPOC>12471280947</IPOC>
    pugi::xml_node ipocNode = senNode.insert_child_after("IPOC", dioNode);
    mutexIPOC.lock();
      ipocNode.append_child(pugi::node_pcdata).set_value(mFilaIPOC.front().c_str());
      mFilaIPOC.pop();
    mutexIPOC.unlock();
    // </Sen>    
    
    std::cout << "XML RSI salvo: " << doc.save_file("config/RSI.xml") << std::endl;

    // Resposta ao cliente
    doc.print(std::cout);  
    std::string RSI(std::istreambuf_iterator<char>(std::ifstream("config/RSI.xml").rdbuf()), std::istreambuf_iterator<char>());  

    if((n=sendto(sockfd, RSI.c_str(), strlen(RSI.c_str()), 0, (struct sockaddr *)&cliaddr,len)) < 0)
      perror("servidor: erro enviando dados para cliente");

    auto tempoFinal = (cv::getTickCount() - tempoInicial) / cv::getTickFrequency();
    std::cout << "RSI gerado e enviado em " << tempoFinal << " ms." << std::endl;
  }
}
