#ifndef VISAGE_PROG_HPP
#define VISAGE_PROG_HPP

#include <VISAGE/mensurium.hpp>

#include <mutex>
#include <queue>


class Programa
{
    public:
                        Programa(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);
        void executar(cv::Mat& imgR);
        void Manipular();
        void MoverPara(double deltax, double deltay, double deltaz, double vel = 1.f);
        void Rotacionar(double deltaA,double deltaB,double deltaC);
        void inic(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);
        void IniciarCaptura();


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


    public:
        mensuriumAGE            mMensurium;
        bool                    mAproximando;
};

#endif // VISAGE_PROG_HPP
