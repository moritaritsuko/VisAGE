#ifndef VISAGE_PROG_HPP
#define VISAGE_PROG_HPP

#include <VISAGE/mensurium.hpp>


class Programa
{
    public:
                        Programa(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);
        void executar(cv::Mat& imgR);
        void Manipular();
        void MoverPara(double deltax, double deltay, double deltaz, double vel = 1.f);
        void Rotacionar(double deltaA,double deltaB,double deltaC);
        void inic(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);

    private:        
        Mensurium    	mMensurium;

};

#endif // VISAGE_PROG_HPP
