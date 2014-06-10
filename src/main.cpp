#include <VIAGE/prog.hpp>

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <sched.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <fcntl.h>

void uso()
{
    std::cerr << "-u   :  [U]so, imprime esta mensagem" << std::endl;
    std::cerr << "-l   :  [L]argura do tabuleiro (L >= 2)" << std::endl;
    std::cerr << "-a   :  [A]ltura do tabuleiro (A >= 2 & A != L)" << std::endl;
    std::cerr << "-t   :  [T]amanho do quadrado em milimetros (T >= 20.0)" << std::endl;
    std::cerr << "-c   :  [C]âmera a ser utilizada (C >= 0)" << std::endl;
    std::cerr << "-p   :  [P]orta para comunicação RSI (1024 <= P <= 32767)" << std::endl;
    std::cerr << "-r   :  P[r]ioridade (1 <= r <= 99)" << std::endl;
}

int main(int argc, char** argv)
{  
    int opcao;
    unsigned int l = 9, a = 6, c = 0, p = 6008, r = 99;
    float t = 25.4f;
    bool valoresPadrao = true;

    if (argc > 1) valoresPadrao = false;

    opterr = 0;
    while ((opcao = getopt(argc, argv, "ul:a:t:c:p:r:")) != -1)
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
            case 'c':
                c = (unsigned int) atoi(optarg);
                break;
            case 'p':
                p = (unsigned int) atoi(optarg);
                break;
            case 'r':
                r = (unsigned int) atoi(optarg);
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
            c = 0u;
            p = 6008u;
            r = 99u;
        }            
        else if ((l < 2u) || (a < 2u || a == l) || (t < 20.f) || c < 0u || (p < 1024u || p > 32767u) || (r < 1u || r > 99u)) 
        {
            uso();
            return 1;
        }
    }

    /*struct sched_param sched;

    if(sched_getparam(0,&sched)==-1)
                perror("Falha em sched_getparam");    

    sched.sched_priority = r;

    // int sched_setscheduler(pid_t pid, int policy, const struct  sched_param *p);
    if (sched_setscheduler(0, SCHED_FIFO, &sched) == -1) 
    {
        perror("Falha em sched_setscheduler");
        exit(0);
    }

    if (sched_getparam(0, &sched) == -1)
        perror("Falha em sched_getparam");

    switch (sched_getscheduler(0)) 
    {
        case SCHED_FIFO:
                printf("\nScheduler: SCHED_FIFO. Priority: %d\n",sched.sched_priority);
                break;
        case SCHED_RR:
                printf("\nScheduler: SCHED_RR. Priority: %d\n",sched.sched_priority);
                break;
        case SCHED_OTHER:
                printf("\nScheduler: SCHED_OTHER. Priority: %d\n",sched.sched_priority);
                break;
    }*/

    try
    {
        Programa prog(l, a, t, c, p);
        prog.executar();
    }
    catch (std::exception& e)
    {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
	    return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
