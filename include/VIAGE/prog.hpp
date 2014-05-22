#ifndef VIAGE_PROG_HPP
#define VIAGE_PROG_HPP

#include <VIAGE/mensuriumage.hpp>


class Programa
{
    public:
                        Programa(unsigned int largura, unsigned int altura, float tamanho);
        void            executar();

    private:
        unsigned int    largura;
        unsigned int    altura;
        float           tamanho;
        mensuriumAGE    mAGE;
};

#endif // VIAGE_PROG_HPP
