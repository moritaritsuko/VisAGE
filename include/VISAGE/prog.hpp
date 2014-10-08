#ifndef VISAGE_PROG_HPP
#define VISAGE_PROG_HPP

#include <VISAGE/mensurium.hpp>

#include <opencv2/opencv.hpp>

#include <mutex>
#include <queue>


class Programa
{
    public:
                        Programa(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);
        void executar(cv::Mat& imgR);
        void Manipular();
        void MoverPara(double deltax = 0.f, double deltay = 0.f, double deltaz = 0.f, double vel = 1.f);
        void Rotacionar(double deltaA = 0.f,double deltaB = 0.f,double deltaC = 0.f, double vel = 0.1f);
        void GAMAG();
        void ativarGAMAG();
        void desativarGAMAG();
        void inic(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);
        void IniciarCaptura();
        void CapturaCameraMono();


    private:        
        void*                   CapturarImagem(void);
        static void*            chamarCapturarImagem(void *arg){return ((Programa*)arg)->CapturarImagem();}


    private:
        std::queue<Marcador>    filaMarcadores;
        std::mutex              mutexMarcador;
        std::queue<cv::Mat>     filaImagens;
        std::mutex              mutexImagem;
        unsigned int            largura;
        unsigned int            altura;
        float                   tamanho;
        unsigned int            camera;
        unsigned int            controleGAMAG;


    public:
        mensuriumAGE            mMensurium;
        bool                    mAproximando;
        cv::VideoCapture        cap;
};

#endif // VISAGE_PROG_HPP
