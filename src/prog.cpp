#include <VIAGE/prog.hpp>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <iostream>


Programa::Programa(unsigned int l, unsigned int a, float t)
: largura(l)
, altura(a)
, tamanho(t)
, mConect(8001)
, mAGE()
{    
    std::cout << "Largura: " << largura << std::endl;
    std::cout << "Altura: " << altura << std::endl;
    std::cout << "Tamanho (mm): " << tamanho << std::endl;
    mConect.RSI_XML();
}

void Programa::executar()
{
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) 
        exit(EXIT_FAILURE); // TODO: disparar exceção

    cv::Mat imagem;
    int tecla;
    
    while (true)
    {
        tecla = cv::waitKey(30);
        if((tecla & 255) == 27)
            break;

        cap >> imagem;
        mAGE.AcharCentro1Tab(imagem, largura, altura, tamanho);
    }
}
