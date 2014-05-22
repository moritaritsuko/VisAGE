#include <VIAGE/prog.hpp>

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <unistd.h>


void uso()
{
    std::cerr << "-u    :  [U]so, imprime esta mensagem" << std::endl;
    std::cerr << "-l L  :  [L]argura do tabuleiro (L >= 2)" << std::endl;
    std::cerr << "-a A  :  [A]ltura do tabuleiro (A >= 2 & A != L)" << std::endl;
    std::cerr << "-t T  :  [T]amanho do quadrado em milimetros (T >= 20.0)" << std::endl;
}

int main(int argc, char** argv)
{    
    int opcao;
    unsigned int l = 9, a = 6;
    float t = 25.4f;
    bool valoresPadrao = true;

    if (argc > 1) valoresPadrao = false;

    opterr = 0;
    while ((opcao = getopt(argc, argv, "ul:a:t:")) != -1)
    {
        switch (opcao)
        {
            case 'u':
                uso();
                return 0;
            case 'l':
                l = (unsigned int) atoi(optarg);
                break;
            case 'a':
                a = (unsigned int) atoi(optarg);
                break;
            case 't':
                t = atof(optarg);
                break;
            case '?':
                uso();
                return 1;
            default:
                abort();
        }

        if (valoresPadrao)
        {
            l = 9u;    
            a = 6u;   
            t = 25.4f;
        }            
        else if ((l < 2u) || (a < 2u || a == l) || (t < 20.f)) 
        {
            uso();
            return 1;
        }
    }

    try
    {
        Programa prog(l, a, t);
        prog.executar();
    }
    catch (std::exception& e)
    {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
	    return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
