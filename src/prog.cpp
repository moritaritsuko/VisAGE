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
    const double setPX = 37.5;
    const double setPY = -160.5;
    
    while (true)
    {
        int tecla = cv::waitKey(30);
        if((tecla & 255) == 27)
            break;

        cv::Mat img;      
        if (!mAGE.filaImagens.empty())
        {          
            mAGE.mutexImagem.lock();
                img = mAGE.filaImagens.front();
                mAGE.filaImagens.pop();
            mAGE.mutexImagem.unlock();
        }

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

            auto x = posicao.at<double>(0, 0);
            auto y = posicao.at<double>(1, 0);
            auto z = posicao.at<double>(2, 0);
            auto a = orientacao.at<double>(0, 0);
            auto b = orientacao.at<double>(1, 0);
            auto c = orientacao.at<double>(2, 0);
            double deltaX = setPX - x;
            double deltaY = setPY - y;

            if(deltaY < -1.f || deltaY > 1.f)
            {
                if(deltaX > 0.f)
                {
                    cv::putText(img, cv::format("Dir: <"), cv::Point(10, 45), 1, 1, cv::Scalar(255,0,255));
                    mConect.RSI_XML(0.1f);
                }
                else if(deltaX < 0.f)
                {
                    cv::putText(img, cv::format("Dir: >"), cv::Point(10, 45), 1, 1, cv::Scalar(255,0,255));
                    mConect.RSI_XML(-0.1f);
                }
            }
            else
            {
                cv::putText(img, cv::format("Dir: CENTRO"), cv::Point(10, 45), 1, 1, cv::Scalar(255,0,255));
                mConect.RSI_XML();
            }            
            cv::putText(img, cv::format("Delta X: %f", deltaX), cv::Point(10, 15), 1, 1, cv::Scalar(255,0,255));
            cv::putText(img, cv::format("Delta Y: %f", deltaY), cv::Point(10, 30), 1, 1, cv::Scalar(255,0,255));
        }
        if (!img.empty())
            cv::imshow("AcharTab", img);        
    }
}
