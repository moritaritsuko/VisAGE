#include <VIAGE/prog.hpp>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <cassert>
#include <iostream>


Programa::Programa(unsigned int l, unsigned int a, float t, unsigned int c, unsigned int p)
: mConect(p)
, mAGE(l, a, t, c)
{    
    std::cout << "Largura: " << l << std::endl;
    std::cout << "Altura: " << a << std::endl;
    std::cout << "Tamanho (mm): " << t << std::endl;
    std::cout << "Camera: " << c << std::endl;
    std::cout << "Porta RSI: " << p << std::endl;
}

void Programa::executar()
{
    int tecla;
    
    while (true)
    {
        tecla = cv::waitKey(30);
        if((tecla & 255) == 27)
            break;

        if (mAGE.filaMarcadores.empty())
            mConect.RSI_XML();
        else
        {
            Marcador marco;
            mAGE.mutexMarcador.lock();
                marco = mAGE.filaMarcadores.front();
                mAGE.filaMarcadores.pop();
            mAGE.mutexMarcador.unlock();
            auto posicao = marco.getPosicao();
            auto orientacao = marco.getOrientacao();

            double setPX = 37.5;
            double setPY = -160.5;

            auto x = posicao.at<double>(0, 0);
            auto y = posicao.at<double>(1, 0);
            auto z = posicao.at<double>(2, 0);
            auto a = orientacao.at<double>(0, 0);
            auto b = orientacao.at<double>(1, 0);
            auto c = orientacao.at<double>(2, 0);
            double deltaX = setPX - x;
            double deltaY = setPY - y;
            //std::cout << "X = " << x << " | Y = " << y << " | Z = " << z << " | A = " << a << " | B = " << b << " | C = " << c << std::endl;
            //std::cout << "Delta X = " << deltaX << " | Delta Y = " << deltaY << std::endl;
            if(deltaY < -1.f || deltaY > 1.f){
                if(deltaX > 0.f){
                    //std::cout << "<" << std::endl;
                    mConect.RSI_XML(0.1f);
                }
                else if(deltaX < 0.f){
                    //std::cout << ">" << std::endl;
                    mConect.RSI_XML(-0.1f);
                }
            }
            else
            {
                //std::cout << "centro!" << std::endl;
                while (!mAGE.filaMarcadores.empty())
                    mAGE.filaMarcadores.pop();
                mConect.RSI_XML();
            }

            /*mConect.RSI_XML(posicao.at<double>(0, 0), posicao.at<double>(1, 0), posicao.at<double>(2, 0), 
                orientacao.at<double>(0, 0), orientacao.at<double>(1, 0), orientacao.at<double>(2, 0));*/
        }
    }
}
