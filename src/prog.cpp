#include <VIAGE/prog.hpp>
#include <VIAGE/mensuriumage.hpp>

#include <iostream>


Programa::Programa(unsigned int l, unsigned int a, float t)
: largura(l)
, altura(a)
, tamanho(t)
, mAGE()
{
    std::cout << "Largura: " << largura << std::endl;
    std::cout << "Altura: " << altura << std::endl;
    std::cout << "Tamanho (mm): " << tamanho << std::endl;
}

void Programa::executar()
{

}
