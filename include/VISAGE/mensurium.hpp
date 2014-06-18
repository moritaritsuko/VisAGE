#ifndef VISAGE_MENSURIUMAGE_HPP
#define VISAGE_MENSURIUMAGE_HPP

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <queue>
#include <mutex>

class Marcador{
 public:
    void Inic(int x, int y,float dx,float dy);
    Marcador();
    CvPoint* CentroTab(std::vector<cv::Point2f> pontos);
    void calcCantosDigonal(std::vector<cv::Point2f> C);
    CvPoint* getCantosDigonal();
    cv::Mat getPosicao();
    void setPosicao(cv::Mat pos);
    cv::Mat getOrientacao();
    void setOrientacao(cv::Mat orient);
    cv::Mat getMatP3D();
    bool VerificaCor(cv::Mat& img, cv::Scalar cor,cv::Scalar deltaCor );
    void AcharCantoProx(cv::Mat src, int deltaVan, cv::Mat imgDes);
    cv::Point getCentroImg();

    void setValido();
    bool isValido();

 private:
    CvPoint centroImg;
    CvPoint cantosDigonal[4];
    cv::Mat orientacao;
    cv::Mat posicao;
    CvPoint cantoProximo;
    CvMat*   posCantoProximo;
    double  tamanhoReal[2];
    double  tamanhoDig;
    CvSize  tamTab;
    cv::Mat matPontos3D;
    std::vector<cv::Point2f> corners;
    int cornerCount;
    double deltaTab[2];
    cv::Mat PontosTab3D();
    double Dist(CvPoint p1, CvPoint p2);
    cv::Point canto;

    bool valido;
};

class Placa{
  public:
    Placa();
    Marcador* marco;
    void Inic(int n);
    void CalcentroPlaca();
    void setarPontosCam();
    void setnMarcoAch(int nMarcoAch);
    int getnMarcoAch();
    cv::Point getPosCentroImg();
    cv::Mat getmatPontosPlaca3D();
    Marcador getMarcador(int i);
  private:
    CvMat posicaoCentro;
    CvMat rotacao;
    cv::Point posCentroImg;
    CvMat* posCantos[];
    CvMat* pontosPlaca3D;
    cv::Mat matPontosPlaca3D;
    CvMat* pontosPlacaCam;
    int nMarcoAch;
};

class Mensurium
{
public:
    Mensurium(unsigned int l, unsigned int a, float t, unsigned int c);
    int AcharTabs(cv::Mat img, int n, CvMat** trans, int npl, cv::Mat imgDes = cv::Mat(0,0,CV_8UC1));
    Marcador AcharCentro1Tab(cv::Mat img, Marcador& marco, unsigned int largura, unsigned int altura, float tamanho);
    bool Rodar(char *nomeJan, cv::Mat img);
    cv::Mat Stereo(cv::Mat imgE,cv::Mat imgD);
    cv::Mat steroRegMarcos();
    Placa getPlaca(int i);
    Placa* placa;
    void IniciarCaptura();
    std::queue<Marcador> filaMarcadores;
    std::mutex mutexMarcador;
    std::queue<cv::Mat> filaImagens;
    std::mutex mutexImagem;


private:

    cv::Mat distCoeffs;
    cv::Mat cameraMatrix;
    unsigned int largura;
    unsigned int altura;
    float        tamanho;
    unsigned int camera;

    void* CapturarImagem(void);
    static void*  chamarCapturarImagem(void *arg){return ((Mensurium*)arg)->CapturarImagem();}

};

#endif // VISAGE_MENSURIUMAGE_HPP
