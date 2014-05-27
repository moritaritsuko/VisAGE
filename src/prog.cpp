#include <VIAGE/prog.hpp>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <cassert>
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
    mConect.CriarConexao();
    mConect.IniciarLeitura();
}

void Programa::executar()
{
    cv::VideoCapture cap(0);
    assert(cap.isOpened());

    cv::Mat imagem;
    int tecla;
    
    while (true)
    {
        tecla = cv::waitKey(30);
        if((tecla & 255) == 27)
            break;


        cap >> imagem;
        Marcador marco;
        mAGE.AcharCentro1Tab(imagem, marco, largura, altura, tamanho);

        if (marco.isValido())
        {
            // TODO: gerar XML com dados do marco; estabelecer conexão com robô e responder mensagem;
            auto posicao = marco.getPosicao();
            auto orientacao = marco.getOrientacao();
            mConect.RSI_XML(posicao.at<double>(0, 0), posicao.at<double>(0, 1), posicao.at<double>(0, 2), 
                orientacao.at<double>(0, 0), orientacao.at<double>(0, 1), orientacao.at<double>(0, 2));
        }
    }
}
