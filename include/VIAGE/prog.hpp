#ifndef VIAGE_PROG_HPP
#define VIAGE_PROG_HPP

#include <VIAGE/conectrobo.hpp>
#include <VIAGE/mensuriumage.hpp>


class Programa
{
    public:
                        Programa(unsigned int largura, unsigned int altura, float tamanho, unsigned int camera, unsigned int porta);
        void            executar();

    private:        
        ConectRobo      mConect;
        mensuriumAGE    mAGE;

};

#endif // VIAGE_PROG_HPP
