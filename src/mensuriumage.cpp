#include <VIAGE/mensuriumage.hpp>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <iostream>
#include <cmath>


void Marcador::Inic(int x, int y,float dx,float dy)
{
    deltaTab[0] = dx;
    deltaTab[1] = dy;
    tamTab = cvSize(x, y);
    tamanhoReal[0] = tamTab.width*dx;
    tamanhoReal[1] = tamTab.height*dy;
    tamanhoDig = sqrt(pow(tamanhoReal[0], 2)+pow(tamanhoReal[1], 2));
//    pontosTab3D = PontosTab3D();
    matPontos3D = PontosTab3D();
//    corners = new CvPoint2D32f[tamTab.width * tamTab.height];//cvPoint2D32f(tamTab.width , tamTab.height);
    cornerCount =  tamTab.width * tamTab.height;
}

Marcador::Marcador()
{
  valido = false;
}

cv::Mat Marcador::PontosTab3D(){
//    CvMat* pontos = cvCreateMat(tamTab.height,tamTab.width, CV_32FC1);
    cv::Mat mat(tamTab.width*tamTab.height,3, CV_32FC1);
    for(int h = 0; h < tamTab.height; h++){
     for(int w = 0; w < tamTab.width; w++){
//            cvmSet(pontos,w+tamTab.width*h, 0, w*deltaTab[0]);
//            cvmSet(pontos,w+tamTab.width*h, 1, h*deltaTab[1]);
//            cvmSet(pontos,w+tamTab.width*h, 2, 0.0f);
            mat.at<float>(w+tamTab.width*h,0) = (float)w*deltaTab[0];
            mat.at<float>(w+tamTab.width*h,1) = (float)h*deltaTab[1];
            mat.at<float>(w+tamTab.width*h,2) = (float)0.0f;
        }
       }

    return mat;
}

cv::Mat Marcador::getMatP3D(){
    return matPontos3D;
}

double Marcador::Dist(CvPoint p1, CvPoint p2){
        return sqrt( pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
    }

CvPoint* Marcador::CentroTab(std::vector<cv::Point2f> pontos){

    CvPoint centro;
    CvPoint centro1;
    CvPoint centro2;
    CvPoint centro3;
    cornerCount = pontos.size();


//    if(op){
    if(cornerCount==4){
        centro  = cvPoint((int)pontos[0].x,(int)pontos[0].y);
        centro1 = cvPoint((int)pontos[1].x,(int)pontos[1].y);
        centro2 = cvPoint((int)pontos[2].x,(int)pontos[2].y);
        centro3 = cvPoint((int)pontos[3].x,(int)pontos[3].y);
    }else{
     if(tamTab.width==4){
      centro  = cvPoint((int)pontos[0].x,(int)pontos[0].y);
      centro1 = cvPoint((int)pontos[3].x,(int)pontos[3].y);
      centro2 = cvPoint((int)pontos[cornerCount-4].x,(int)pontos[cornerCount-4].y);
      centro3 = cvPoint((int)pontos[cornerCount-1].x,(int)pontos[cornerCount-1].y);
     }else{
      centro  = cvPoint((int)pontos[0].x,(int)pontos[0].y);
      centro1 = cvPoint((int)pontos[5].x,(int)pontos[5].y);
      centro2 = cvPoint((int)pontos[cornerCount-6].x,(int)pontos[cornerCount-6].y);
      centro3 = cvPoint((int)pontos[cornerCount-1].x,(int)pontos[cornerCount-1].y);
     }
    }
//    }else{
//        CvPoint* pontostmp = (CvPoint*)pontos;
//        centro =  pontostmp[0];
//        centro1 = pontostmp[1];
//        centro2 = pontostmp[2];
//        centro3 = pontostmp[3];
//    }



    double dist[6];
    dist[0] = Dist(centro,centro1);
    dist[1] = Dist(centro,centro2);
    dist[2] = Dist(centro,centro3);
    dist[3] = Dist(centro1,centro2);
    dist[4] = Dist(centro1,centro3);
    dist[5] = Dist(centro2,centro3);

    double max = 0;
    int maxI = 0;
    for(int i = 0;i<6;i++){
      if(dist[i]>max){
         max = dist[i];
          maxI = i;
      }
    }

    CvPoint dig1[2];
    CvPoint dig2[2];
    switch (maxI) {
    case 0:
        dig1[0] = centro;dig1[1] = centro1;
        dig2[0] = centro2;dig2[1] = centro3;
        break;

    case 1:
        dig1[0] = centro;dig1[1] = centro2;
        dig2[0] = centro1;dig2[1] = centro3;
    break;

    case 2:
        dig1[0] = centro;dig1[1] = centro3;
        dig2[0] = centro1;dig2[1] = centro2;
    break;

    case 3:
        dig1[0] = centro1;dig1[1] = centro2;
        dig2[0] = centro;dig2[1] = centro3;
    break;

    case 4:
        dig1[0] = centro1;dig1[1] = centro3;
        dig2[0] = centro;dig2[1] = centro2;
    break;

    case 5:
        dig1[0] = centro2;dig1[1] = centro3;
        dig2[0] = centro;dig2[1] = centro1;
    break;



    default:
        break;
    }

        double a[2];
        double b[2];

        CvMat* A = cvCreateMat(2,2,CV_32FC1);
        CvMat* B = cvCreateMat(2,1,CV_32FC1);;

        cvmSet(A,0, 0, dig1[0].x);
        cvmSet(A,0, 1, 1);
        cvmSet(A,1, 0, dig1[1].x);
        cvmSet(A,1, 1, 1);

        cvmSet(B,0, 0, dig1[0].y);
        cvmSet(B,1, 0, dig1[1].y);

        CvMat* X = cvCreateMat(2,1,CV_32FC1);
        cvSolve(A, B, X);

        a[0] = cvmGet(X,0,0);
        b[0] = cvmGet(X,1,0);

        cvmSet(A,0, 0, dig2[0].x);
        cvmSet(A,1, 0, 1);
        cvmSet(A,1, 0, dig2[1].x);
        cvmSet(A,1, 1, 1);

        cvmSet(B,0, 0, dig2[0].y);
        cvmSet(B,1, 0, dig2[1].y);

        cvSolve(A, B, X);

        a[1] = cvmGet(X,0,0);
        b[1] = cvmGet(X,1,0);

        int x =(int) ((b[1]-b[0])/(a[0]-a[1]));
        int y = (int) (a[0]*x+b[0]);

        CvPoint centroP  = cvPoint(x,y);
        CvPoint* p = new CvPoint[5];
            p[0] = centroP;
            p[1] = dig1[0];
            p[2] = dig1[1];
            p[3] = dig2[0];
            p[4] = dig2[1];


return p;

}

void Marcador::calcCantosDigonal(std::vector<cv::Point2f> C){
    CvPoint* ptmp = CentroTab(C);
    cornerCount = C.size();
    corners = C;
    centroImg = ptmp[0];
    cantosDigonal[0] = ptmp[1];
    cantosDigonal[1] = ptmp[2];
    cantosDigonal[2] = ptmp[3];
    cantosDigonal[3] = ptmp[4];

}


//CvMat* Marcador::getPontosTab3D() {
//            return pontosTab3D;
//}

CvPoint* Marcador::getCantosDigonal() {
            return cantosDigonal;
}

cv::Mat Marcador::getPosicao() 
{
            return posicao;
}

void Marcador::setPosicao(cv::Mat pos) 
{
  posicao = pos;
}

cv::Mat Marcador::getOrientacao()
{
  return orientacao;
}

void Marcador::setOrientacao(cv::Mat orient)
{
  orientacao = orient;
}

bool Marcador::VerificaCor(cv::Mat& img, cv::Scalar cor, cv::Scalar deltaCor){

    cv::Mat roi(img,cv::Rect(cantosDigonal[0],cantosDigonal[1]));

    cv::Mat imgHSV;
    cv::cvtColor(roi, imgHSV, CV_RGB2HSV);

    cv::Mat imgThresh;
    cv::inRange(imgHSV, cor, deltaCor, imgThresh);


    if(cv::countNonZero(imgThresh)>(int)(0.1*roi.cols*roi.rows)){
        return true;
    }

    return false;


}

void Marcador::AcharCantoProx(cv::Mat src, int deltaVan,cv::Mat imgDes){

    int xi = centroImg.x-deltaVan;
    int yi = centroImg.y-deltaVan;
    int xf = centroImg.x+deltaVan;
    int yf = centroImg.y+deltaVan;

    cv::Mat imgC;
    src.copyTo(imgC);


    if(xi<0) xi = 0;
    if(yi<0) yi = 0;
    if(xf>src.cols) xf = src.cols;
    if(yf>src.rows) yf = src.rows;


    cv::Point pc1(xi,yi);
//    std::cout<<pc1<<std::endl;
    cv::Point pc2(xf,yf);
//    std::cout<<pc2<<std::endl;

    cv::circle(src,cv::Point(centroImg.x,centroImg.y),Dist(cantosDigonal[0],cantosDigonal[1])/2,cv::Scalar(0,0,0,0),50,15,0);
    cv::Mat roi(src,cv::Rect(pc1,pc2));
    cv::Mat roiDes(imgDes,cv::Rect(pc1,pc2));


    cv::Mat imgCanny;

    imgCanny.create(roi.rows,roi.cols,CV_8UC1);
    cv::Mat imgCinza(roi.rows,roi.cols,CV_8UC1);



    cv::cvtColor( roi, imgCinza, CV_BGR2GRAY );

     cv::blur( imgCinza, imgCanny, cv::Size(5,5) );

     cv::Canny( imgCinza, imgCanny, 50, 170);
//     cv::imshow("canny",imgCanny);
//     cv::waitKey();

     cv::Point ptTmp[4];
     int cp = 0;

     std::vector<cv::Vec4i> lines;
     cv::HoughLinesP( imgCanny, lines, 1.1, CV_PI/180, 80, 70, 10 );
      //std::cout<<"N linhas= "<<lines.size()<<std::endl;
     for( size_t i = 0; i < lines.size(); i++ )
     {
         cv::line( roiDes, cv::Point(lines[i][0], lines[i][1]),
             cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 1, 8 );

                   if(i<2 && lines.size()==2){

                       ptTmp[cp] = cv::Point(lines[i][0], lines[i][1]);
                       ptTmp[cp+1] =  cv::Point(lines[i][2], lines[i][3]);
                       cp += 2;
                   }
     }

      if(lines.size()==2){
        int a1 = (ptTmp[0].x*ptTmp[1].y) - (ptTmp[0].y*ptTmp[1].x);

        int b1 = ptTmp[2].x - ptTmp[3].x;

        int c1 = a1*b1;

        int a2 = ptTmp[2].x*ptTmp[3].y - ptTmp[2].y*ptTmp[3].x;

        int b2 = ptTmp[0].x - ptTmp[1].x;

        int c2 = a2*b2;

        int d1 = ptTmp[2].y-ptTmp[3].y;

        int d2 = ptTmp[0].y-ptTmp[1].y;

        int c3 = d1*a1;

        int c4 = d2*a2;

        float dem = (b2*d1)-(b1*d2);

        float x = (c1-c2)/dem;
        float y = (c3-c4)/dem;


         cv::Point ip(x,y);

         //std::cout<<"ip= "<<ip<<std::endl;

         cv::circle(roiDes,ip,2,cv::Scalar(255,255,0),2);
      }

imgC.copyTo(src);

}

cv::Point Marcador::getCentroImg(){
    return cv::Point(centroImg.x,centroImg.y);
}

void Marcador::setValido()
{
  valido = true;
}

bool Marcador::isValido()
{
  return valido;
}

Placa::Placa(){
//    marco = new Marcador[4];
}

void Placa::Inic(int n){

    nMarcoAch = 0;
    pontosPlaca3D = cvCreateMat(4, 3, CV_32FC1);
    pontosPlacaCam = cvCreateMat(4, 2, CV_32FC1);
    matPontosPlaca3D.create(4, 2, CV_32FC1);

    marco = new Marcador[n];

               for(int i =0;i<n;i++){
                   marco[i].Inic(4, 4, 15.0f, 15.0f);
               }
                           cvmSet(pontosPlaca3D,0,0,20.0f);
                           cvmSet(pontosPlaca3D,0,1,20.0f);
                           cvmSet(pontosPlaca3D,0,2,0.0f);

                           cvmSet(pontosPlaca3D,1,0,20.0f);
                           cvmSet(pontosPlaca3D,1,1,490.f);
                           cvmSet(pontosPlaca3D,1,2,0.0f);

                           cvmSet(pontosPlaca3D,2,0,1180.f);
                           cvmSet(pontosPlaca3D,2,1,490.f);
                           cvmSet(pontosPlaca3D,2,2,0.0f);

                           cvmSet(pontosPlaca3D,3,0,1180.f);
                           cvmSet(pontosPlaca3D,3,1,20.0f);
                           cvmSet(pontosPlaca3D,3,2,0.0f);

                           matPontosPlaca3D.at<float>(0,0) = 20.f;
                           matPontosPlaca3D.at<float>(0,1) = 20.f;
                           matPontosPlaca3D.at<float>(0,2) = 0.f;

                           matPontosPlaca3D.at<float>(1,0) = 20.f;
                           matPontosPlaca3D.at<float>(1,1) = 490.f;
                           matPontosPlaca3D.at<float>(1,2) = 0.f;

                           matPontosPlaca3D.at<float>(2,0) = 1180.f;
                           matPontosPlaca3D.at<float>(2,1) = 490.f;
                           matPontosPlaca3D.at<float>(2,2) = 20.f;

                           matPontosPlaca3D.at<float>(3,0) = 1180.f;
                           matPontosPlaca3D.at<float>(3,1) = 20.f;
                           matPontosPlaca3D.at<float>(3,2) = 0.f;



}

void Placa::CalcentroPlaca(){
    std::vector<cv::Point2f> posM;
    posM.push_back(cv::Point2f(marco[0].getCentroImg().x,marco[0].getCentroImg().y));
    posM.push_back(cv::Point2f(marco[1].getCentroImg().x,marco[1].getCentroImg().y));
    posM.push_back(cv::Point2f(marco[2].getCentroImg().x,marco[2].getCentroImg().y));
    posM.push_back(cv::Point2f(marco[3].getCentroImg().x,marco[3].getCentroImg().y));
    CvPoint* ptmp = marco[0].CentroTab(posM);
    posCentroImg = cv::Point(ptmp->x,ptmp->y);
}

cv::Mat Placa::getmatPontosPlaca3D(){
    return matPontosPlaca3D;
}

void Placa::setarPontosCam(){
 cvmSet(pontosPlacaCam,0,0,marco[0].getCentroImg().x);
 cvmSet(pontosPlacaCam,0,1,marco[0].getCentroImg().y);

  cvmSet(pontosPlacaCam,1,0,marco[1].getCentroImg().x);
  cvmSet(pontosPlacaCam,1,1,marco[1].getCentroImg().y);

  cvmSet(pontosPlacaCam,2,0,marco[2].getCentroImg().x);
  cvmSet(pontosPlacaCam,2,1,marco[2].getCentroImg().y);

  cvmSet(pontosPlacaCam,3,0,marco[3].getCentroImg().x);
  cvmSet(pontosPlacaCam,3,1,marco[3].getCentroImg().y);

}

void Placa::setnMarcoAch(int nMarcoAch) {
            this->nMarcoAch = nMarcoAch;
}

cv::Point Placa::getPosCentroImg(){
    return posCentroImg;
}

int Placa::getnMarcoAch(){
    return nMarcoAch;
}

int  mensuriumAGE::AcharTabs(cv::Mat img, int n, CvMat **trans, int npl,cv::Mat imgDes){


    CvPoint p[n][3];

    cv::Mat imgC;
    img.copyTo(imgC);

    cv::Mat cinza = cv::Mat(imgC.cols, imgC.rows, CV_8SC1);
    cv::Mat pb = cv::Mat(imgC.cols, imgC.rows, CV_8SC1);

    cv::cvtColor(imgC, cinza, CV_BGR2GRAY);
//    cv::threshold(cinza, pb, 72,255 , CV_THRESH_BINARY);// 3 = 70 =72
    cv::adaptiveThreshold(cinza,pb,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
    int nAchado = 0;
    trans = new CvMat*[n];
    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);

    for(int i =0;i<n;i++){
     //Thread.sleep(150);
     CvSize patternSize = cvSize(4, 4);//CvSize(6, 8);cel 6, 9
     std::vector<cv::Point2f> corners;
     bool found = cv::findChessboardCorners( pb, patternSize, corners, CV_CALIB_CB_NORMALIZE_IMAGE);

     if(found){

//     cv::drawChessboardCorners( imgC, patternSize, cv::Mat(corners), found );
     nAchado++;
//     std::cout<<"c placa= "<<nAchado<<std::endl;
     placa[npl].marco[i].calcCantosDigonal(corners); //CentroTab(cornerCount, corners);

     placa[npl].marco[i].getCentroImg();
      //CvMat* R = cvCreateMat(1, 3, CV_32FC1);
      trans[i] = cvCreateMat(1, 3, CV_32FC1);
      placa[npl].marco[i].setPosicao(trans[i]);

      //placa[npl].marco[i].gettPontosTab3D(), pontosCam
      cv::solvePnP(placa[npl].marco[i].getMatP3D(), corners, cameraMatrix, distCoeffs, rvec, tvec, false);
      //cv::solvePnPRansac(placa[npl].marco[i].getMatP3D(), corners, cameraMatrix, distCoeffs, rvec, tvec, false);
      float dist = sqrt(tvec.at<double>(0,0)*tvec.at<double>(0,0)+tvec.at<double>(1,0)*tvec.at<double>(1,0)+tvec.at<double>(2,0)*tvec.at<double>(2,0));
      if(!imgDes.empty()){
          cv::putText( imgDes,cv::format("Dist: %f",dist), placa[npl].marco[i].getCentroImg()+cv::Point(-100,-100), 1, 2,cv::Scalar(255,0,255));
          cv::putText( imgDes,cv::format("Rot: %f,%f,%f",rvec.at<double>(0,0)*180.f/CV_PI,rvec.at<double>(1,0)*180.f/CV_PI,rvec.at<double>(2,0)*180.f/CV_PI), placa[npl].marco[i].getCentroImg()+cv::Point(-100,-150), 1, 2,cv::Scalar(255,0,255));
      }
      //std::cout<<"Dist["<<i<<"]: "<<dist<<std::endl;

     cv::line(pb, placa[npl].marco[i].getCantosDigonal()[0], placa[npl].marco[i].getCantosDigonal()[1], cv::Scalar(0,0,0), 20, 1, 0);
     cv::line(pb, placa[npl].marco[i].getCantosDigonal()[2], placa[npl].marco[i].getCantosDigonal()[3], cv::Scalar(0,0,0), 20, 1, 0);

     p[i][0] = placa[npl].marco[i].getCentroImg();
     p[i][1] = placa[npl].marco[i].getCantosDigonal()[0];
     p[i][2] = placa[npl].marco[i].getCantosDigonal()[1];


    }

//cv::imshow("AcharTab",pb);

  }

    placa[npl].setnMarcoAch(nAchado);
    return nAchado;
}

mensuriumAGE::mensuriumAGE(unsigned int l, unsigned int a, float t, unsigned int c)
: largura(l)
, altura(a)
, tamanho(t)
, camera(c)
{
    placa = new Placa[2];
    placa[0].Inic(4);
    placa[1].Inic(4);
    cv::FileStorage fs("config/Calib.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    IniciarCaptura();
}

Placa mensuriumAGE::getPlaca(int i){
    return placa[i];
}

Marcador mensuriumAGE::AcharCentro1Tab(cv::Mat img, Marcador& marco, unsigned int largura, unsigned int altura, float tamanho)
{
    cv::Size tTab(largura, altura);
    cv::Mat cinza(img.rows,img.cols,CV_8UC1);
    marco.Inic(largura, altura, tamanho, tamanho);

    cv::cvtColor(img,cinza,CV_RGB2GRAY);
    cv::Mat imgThresh=cv::Mat(img.rows,img.cols,CV_8UC1);
    cv::adaptiveThreshold(cinza,imgThresh,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
    //cv::threshold(cinza,imgThresh,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);
    //cv::imshow("AcharTab",imgThresh);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners( imgThresh, tTab, corners,CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);

    if(found)
    {      
      cv::drawChessboardCorners(img,tTab,corners,found);
      cv::Mat R(3,1,cv::DataType<double>::type);
      cv::Mat trans(3,1,cv::DataType<double>::type);

      marco.calcCantosDigonal(corners);

      cv::solvePnP(marco.getMatP3D(),corners,cameraMatrix,distCoeffs,R,trans,false,0);

      marco.setPosicao(trans);
      marco.setOrientacao(R);

      double dist = sqrt(pow(trans.at<double>(0,0),2)+pow(trans.at<double>(1,0),2)+pow(trans.at<double>(2,0),2));
      //std::cout<<"Dist: "<<dist<<std::endl;
      cv::Point temp = marco.getCentroImg();
      cv::putText( img,cv::format("Dist: %f",dist), temp, 1, 1,cv::Scalar(255,0,255));
      cv::circle(img,temp,5,cv::Scalar(255,0,255),3,2,0);
      cv::circle(img,cvPoint(img.cols/2,img.rows/2),5,cv::Scalar(0,0,255),3,2,0);
      cv::line(img,temp,cvPoint(img.cols/2,img.rows/2),cv::Scalar(0,0,0),3,2,0);
      cv::putText(img,cv::format("Rot: %f,%f,%f",R.at<double>(0,0)*180.f/CV_PI,R.at<double>(1,0)*180.f/CV_PI,R.at<double>(2,0)*180.f/CV_PI), marco.getCentroImg()+cv::Point(-100,-150), 1, 1,cv::Scalar(255,0,255));
      marco.setValido();
    }    

    cv::imshow("AcharTab",img);
    return marco;
}

cv::Mat mensuriumAGE::Stereo(cv::Mat imgE, cv::Mat imgD){

    cv::cvtColor(imgE,imgE,CV_RGB2GRAY);
    cv::cvtColor(imgD,imgD,CV_RGB2GRAY);

    cv::Mat disp;
    cv::StereoBM sbm;
    sbm.state->SADWindowSize = 5;
    sbm.state->numberOfDisparities = 3*16;
    sbm.state->preFilterSize = 5;
    sbm.state->preFilterCap = 60;
    sbm.state->minDisparity = 0;
    sbm.state->textureThreshold = 255;
    sbm.state->uniquenessRatio = 0;
    sbm.state->speckleWindowSize = 255;
    sbm.state->speckleRange = 255;
    sbm.state->disp12MaxDiff = 255;
    sbm(imgE,imgD,disp);

    cv::Mat disp8U;
    double minVal, maxVal;
    minMaxLoc( disp, &minVal, &maxVal );
    disp.convertTo( disp8U, CV_8UC1, 255/(maxVal - minVal));

    return disp8U;
}

bool mensuriumAGE::Rodar(char* nomeJan,cv::Mat img){

    bool resp = false;

    CvMat** trans;
    cv::Scalar cor[4];
    cor[0] = cv::Scalar(0,0,45,0);
    cor[1] = cv::Scalar(0,0,110,0);
    cor[2] = cv::Scalar(0,0,145,0);
    cor[3] = cv::Scalar(0,0,175,0);

     cv::Scalar dcor[4];
     dcor[0] = cv::Scalar(255,255,120,255);
     dcor[1] = cv::Scalar(255,255,150,255);
     dcor[2] = cv::Scalar(255,255,170,255);
     dcor[3] = cv::Scalar(255,255,210,255);

     cv::Mat imgSaida;
     img.copyTo(imgSaida);

    AcharTabs(img,4,trans,0,imgSaida);
    //std::cout<<"n placa= "<<placa[0].getnMarcoAch()<<std::endl;

    if (placa[0].getnMarcoAch()==4){
        placa[0].CalcentroPlaca();
        cv::circle(imgSaida,placa[0].getPosCentroImg(),1,cv::Scalar(255,0,255),2,1,0);


        for(int i= 0;i<4;i++ ){
            placa[0].marco[i].AcharCantoProx(img,150,imgSaida);
          for(int j = 0; j<4; j++){
            if (placa[0].marco[i].VerificaCor(img,cor[j],dcor[j])){
                if(j == 0)cv::putText( imgSaida,cv::format("Azul"), placa[0].marco[j].getCentroImg(), 1, 1,cv::Scalar(255,0,0));
                if(j == 1)cv::putText( imgSaida,cv::format("Vermelho"), placa[0].marco[j].getCentroImg(), 1, 1,cv::Scalar(0,0,255));
                if(j == 2)cv::putText( imgSaida,cv::format("Verde"), placa[0].marco[j].getCentroImg(), 1, 1,cv::Scalar(0,255,0));
                if(j == 3)cv::putText( imgSaida,cv::format("Amarelo"), placa[0].marco[j].getCentroImg(), 1, 1,cv::Scalar(0,255,255));
            }
        }

       }


        resp = true;
    }

    cv::resize(imgSaida,imgSaida,cv::Size(img.cols/2,img.rows/2));

    cv::imshow(nomeJan,imgSaida);
    cv::waitKey(5);

    return resp;
}

void mensuriumAGE::IniciarCaptura()
{
  pthread_t tid;
  int result;
  result = pthread_create(&tid, 0, mensuriumAGE::chamarCapturarImagem, this);
  if (result == 0)
    pthread_detach(tid);
}

void *mensuriumAGE::CapturarImagem(void)
{
  cv::VideoCapture cap(camera);
  assert(cap.isOpened());

  cv::Mat imagem;
  
  while (true)
  {
      cap >> imagem;
      Marcador marco;
      AcharCentro1Tab(imagem, marco, largura, altura, tamanho);

      if (marco.isValido())
      {
        mutexMarcador.lock();
          filaMarcadores.push(marco);
        mutexMarcador.unlock();
      }
  }
}

