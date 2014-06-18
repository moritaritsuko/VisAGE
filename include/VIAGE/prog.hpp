#ifndef VIAGE_PROG_HPP
#define VIAGE_PROG_HPP

#include <VIAGE/conectrobo.hpp>
#include <VIAGE/mensurium.hpp>


class Programa
{
    public:
                        Programa(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);
        void executar(cv::Mat& imgR);
        void Manipular();
        void MoverPara(double deltax, double deltay, double deltaz);
        void Rotacionar(double deltaA,double deltaB,double deltaC);
        void inic(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);

    private:        
        ConectRobo      mConect;
        Mensurium    	mMensurium;

};

#endif // VIAGE_PROG_HPP
