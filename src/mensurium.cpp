#include <VISAGE/mensurium.hpp>

#include "opencv/cv.h"
#include "opencv2/opencv.hpp"
#include "math.h"

#include <algorithm>

void Marcador::Inic(int x, int y,float dx,float dy)
{
    deltaTab[0] = dx;
    deltaTab[1] = dy;
    tamTab = cvSize(x, y);
    tamanhoReal[0] = tamTab.width*dx;
    tamanhoReal[1] = tamTab.height*dy;
    tamanhoDig = sqrt(pow(tamanhoReal[0], 2)+pow(tamanhoReal[1], 2));
    //pontosTab3D = PontosTab3D();
    matPontos3D = cv::Mat(tamTab.width*tamTab.height,3, CV_32FC1);
    matPontos3D = PontosTab3D();
    //    corners = new CvPoint2D32f[tamTab.width * tamTab.height];//cvPoint2D32f(tamTab.width , tamTab.height);
    cornerCount =  tamTab.width * tamTab.height;
    cor = -2;

    posicaoMONO = cv::Mat(3,1,cv::DataType<double>::type);

    orientacao = cv::Mat(3,1,cv::DataType<double>::type);
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

cv::Mat Marcador::getPosicaoMONO() {
    return posicaoMONO;
}

void Marcador::setPosicaoMONO(cv::Mat pos) {
    pos.copyTo(this->posicaoMONO);
}

cv::Point3d Marcador::getPosicaoStereo() {
    return posicaoStereo;
}

void Marcador::setPosicaoStereo(cv::Point3d pos) {
    this->posicaoStereo = pos;
}

CvPoint2D32f* Marcador::getCantosDigonal() {
    return cantosDigonal;
}


cv::Mat Marcador::getMatP3D(){
    return matPontos3D;
}

double Marcador::Dist(CvPoint p1, CvPoint p2){
    return sqrt( pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}

double Marcador::Dist(CvPoint2D32f p1, CvPoint2D32f p2){
    return sqrt( pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}

CvPoint2D32f* Marcador::CentroTab(std::vector<cv::Point2f> pontos){

    CvPoint2D32f centro;
    CvPoint2D32f centro1;
    CvPoint2D32f centro2;
    CvPoint2D32f centro3;
    cornerCount = pontos.size();


    //    if(op){
    if(cornerCount==4){
        centro  = cvPoint2D32f(pontos[0].x,pontos[0].y);
        centro1 = cvPoint2D32f(pontos[1].x,pontos[1].y);
        centro2 = cvPoint2D32f(pontos[2].x,pontos[2].y);
        centro3 = cvPoint2D32f(pontos[3].x,pontos[3].y);
    }else{
        if(tamTab.width==4){
            centro  = cvPoint2D32f(pontos[0].x,pontos[0].y);
            centro1 = cvPoint2D32f(pontos[3].x,pontos[3].y);
            centro2 = cvPoint2D32f(pontos[cornerCount-4].x,pontos[cornerCount-4].y);
            centro3 = cvPoint2D32f(pontos[cornerCount-1].x,pontos[cornerCount-1].y);
        }else{
            centro  = cvPoint2D32f(pontos[0].x,pontos[0].y);
            centro1 = cvPoint2D32f(pontos[5].x,pontos[5].y);
            centro2 = cvPoint2D32f(pontos[cornerCount-6].x,pontos[cornerCount-6].y);
            centro3 = cvPoint2D32f(pontos[cornerCount-1].x,pontos[cornerCount-1].y);
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

    CvPoint2D32f dig1[2];
    CvPoint2D32f dig2[2];
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

    double x =((b[1]-b[0])/(a[0]-a[1]));
    double y = (a[0]*x+b[0]);

    CvPoint2D32f centroP  = cvPoint2D32f(x,y);
    CvPoint2D32f* p = new CvPoint2D32f[5];
    p[0] = centroP;
    p[1] = dig1[0];
    p[2] = dig1[1];
    p[3] = dig2[0];
    p[4] = dig2[1];

    //std::cout<<"centroP = "<<x<<","<<y<<std::endl;


    return p;

}

cv::Point2f* Marcador::calcCantosDigonal(std::vector<cv::Point2f> C){
    CvPoint2D32f* ptmp = CentroTab(C);
    cornerCount = C.size();
    corners = C;
    centroImg = ptmp[0];
    cantosDigonal[0] = ptmp[1];
    cantosDigonal[1] = ptmp[2];
    cantosDigonal[2] = ptmp[3];
    cantosDigonal[3] = ptmp[4];

    pr = new cv::Point2f[4];

    pr[0] = cv::Point2f(ptmp[1]);
    pr[1] = cv::Point2f(ptmp[2]);
    pr[2] = cv::Point2f(ptmp[3]);
    pr[3] = cv::Point2f(ptmp[4]);


    return pr;


}

cv::Point2f Marcador::getCantoProx(){
    return cantoProximo;
}

//CvMat* Marcador::getPontosTab3D() {
//            return pontosTab3D;
//}


cv::Mat Marcador::getOrientacao()
{
    return orientacao;
}

void Marcador::setOrientacao(cv::Mat orient)
{
    orient.copyTo(orientacao) ;
}

bool Marcador::VerificaCor(cv::Mat img, cv::Scalar cor, cv::Scalar deltaCor,int index){

    cv::Mat roi(img,cv::Rect(cv::Point(cantosDigonal[1].x,cantosDigonal[3].y),cv::Point(cantosDigonal[0].x,cantosDigonal[2].y)));

    //    cv::circle(img,cantosDigonal[0],3,cv::Scalar(255,0,0),3);
    //    cv::circle(img,cantosDigonal[1],3,cv::Scalar(0,255,0),3);
    //    cv::circle(img,cantosDigonal[2],3,cv::Scalar(255,255,0),3);
    //    cv::circle(img,cantosDigonal[3],3,cv::Scalar(0,0,255),3);
    //    cv::imshow("imgCor",img);

    cv::Mat matHSV(roi.rows,roi.cols,roi.type());
    cv::cvtColor(roi,matHSV,CV_BGR2HSV);

    cv::Mat matTh(roi.rows,roi.cols,CV_8UC1);
    cv::inRange(matHSV,cor,deltaCor,matTh);

    //        cv::imshow("roi",roi);
    //        cv::imshow("matTh",matTh);
    //        cv::waitKey();

    int total = cv::countNonZero(matTh);

    double res = (double)total/(double)(roi.cols*roi.rows);
    //std::cout<<"Cor"<<index<< "= "<<total<<std::endl;
    //std::cout<<"CorRes"<<index<< "= "<<res<<std::endl;

    //cv::waitKey();

    if(res>=0.3){
        this->cor = index;
        return true;
    }else{
        this->cor = -1;
        return false;
    }
}

int Marcador::VerificaCor(cv::Mat img, cv::Scalar cor[][2], cv::Scalar deltaCor[][2],int nCores){

    cv::Mat roi(img,cv::Rect(cv::Point(cantosDigonal[1].x,cantosDigonal[3].y),cv::Point(cantosDigonal[0].x,cantosDigonal[2].y)));

    cv::Mat matHSV(roi.rows,roi.cols,roi.type());
    cv::cvtColor(roi,matHSV,CV_BGR2HSV);

    int contMax = 0;
    int index = 0;

    for(int i = 0;i<nCores;++i){
        cv::Mat matTh(roi.rows,roi.cols,CV_8UC1);
        cv::inRange(matHSV,cor[i][0],deltaCor[i][1],matTh);

        int total = cv::countNonZero(matTh);

        if(total>contMax){
            index = i;
            contMax = total;
        }
    }

    this->cor = index;

    return index;



}

/*//double DIST(cv::Point p1,cv::Point p2){
//    return (sqrt(pow(p2.x-p1.x,2)+pow(p2.y-p1.y,2)));
//}

//double Angle( cv::Point pt1, cv::Point pt0,cv::Point pt2 ) {
//    double c = DIST(pt0,pt1);
//    double b = DIST(pt0,pt2);
//    double a = DIST(pt1,pt2);

//    double cosA = (-pow(a,2)+pow(b,2)+pow(c,2))/(2*b*c);

//    return 180*acos(cosA)/CV_PI;
//}*/

void Marcador::IdentQRCode(cv::Mat src, int deltaVan){
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
    cv::Point pc2(xf,yf);

    cv::Mat roi(imgC,cv::Rect(pc1,pc2));

    //cv::Mat roi(src,cv::Rect(cv::Point(cantosDigonal[1].x,cantosDigonal[3].y),cv::Point(cantosDigonal[0].x,cantosDigonal[2].y)));

    zbar::ImageScanner scanner;

    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    cv::Mat grey;
    cv::Mat qr = roi;

    cv::imshow("QRimgPre",qr);
    cv::waitKey();

    cv::cvtColor(qr,grey,CV_BGR2GRAY);
    int width = qr.cols;
    int height = qr.rows;
    uchar *raw = (uchar *)grey.data;
    // wrap image data
    zbar::Image image(width, height, "Y800", raw, width * height);
    // scan the image for barcodes
    int n = scanner.scan(image);

    // extract results
    if (n != 0)
    {
        for(zbar::Image::SymbolIterator symbol = image.symbol_begin();
            symbol != image.symbol_end();
            ++symbol)
        {
            std::vector<cv::Point> vp;
            // do something useful with results
            std::cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' <<" "<< std::endl;
            int n = symbol->get_location_size();
            for(int i=0;i<n;i++)
            {
                vp.push_back(cv::Point(symbol->get_location_x(i),symbol->get_location_y(i)));
            }
            cv::RotatedRect r = cv::minAreaRect(vp);
            cv::Point2f pts[4];
            r.points(pts);
            for(int i=0;i<4;i++){
                cv::line(roi,pts[i],pts[(i+1)%4],cv::Scalar(255,0,0),1);
            }
            std::cout<<"Angle: "<<r.angle<<std::endl;
        }
    }else{
        std::cout<<"QR NÃO ENCONTRADO!"<<std::endl;
    }
    //cv::imshow("QRimgPos",qr);
    //cv::waitKey();
}


void Marcador::AcharCantoProx(cv::Mat src, int deltaVan,cv::Mat imgDes){

    bool comCor = false;

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



    cv::Mat  votosCantos = cv::Mat::zeros(xf-xi,yf-yi,CV_64FC1);



    cv::Point pc1(xi,yi);
    cv::Point pc2(xf,yf);

    cv::circle(imgC,cv::Point(centroImg.x,centroImg.y),Dist(cantosDigonal[0],
               cantosDigonal[1])-10,cv::Scalar(30,25,20),CV_FILLED);


    cv::Mat roi(imgC,cv::Rect(pc1,pc2));
    cv::Mat roiDes(imgDes,cv::Rect(pc1,pc2));
    cv::Mat matHSV;
    cv::Mat matHSVTH;

    cv::Mat imgCanny(roi.rows,roi.cols,CV_8UC1);

    if(comCor){

        cv::cvtColor( roi, matHSV, CV_BGR2HSV );

        cv::inRange(matHSV,cv::Scalar(0,0,30),cv::Scalar(250,250,60),matHSVTH);

            cv::imshow("matHSVTH",matHSVTH);

        cv::blur(matHSVTH,matHSVTH,cv::Size(5,5));

        cv::dilate(matHSVTH,matHSVTH,cv::Mat(),cv::Point(0,0),2);

        cv::threshold(matHSVTH,matHSVTH,10,255,CV_THRESH_BINARY);

        cv::erode(matHSVTH,matHSVTH,cv::Mat(),cv::Point(0,0),1);

            cv::imshow("matHSVTHD",matHSVTH);

        cv::Canny(matHSVTH,imgCanny,30,30);

        cv::dilate(imgCanny,imgCanny,cv::Mat(),cv::Point(0,0),2);
    }else{
        cv::Mat imgCinza(roi.rows,roi.cols,CV_8UC1);
        cv::cvtColor( roi, imgCinza, CV_BGR2GRAY );
        cv::blur(imgCinza,imgCinza,cv::Size(5,5));
        cv::Canny(imgCinza,imgCanny,10,10);
        cv::dilate(imgCanny,imgCanny,cv::Mat(),cv::Point(0,0),2);
    }


        //cv::imshow("imgCanny",imgCanny);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(imgCanny, lines, 1.3, CV_PI/180, 150, roi.cols/2, 50 );
    cv::Point ptTmp[4];
    for( size_t j = 0; j < lines.size(); j++ )
        for( size_t i = j; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
                        //cv::line( roiDes, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,255), 1, CV_AA);
            ptTmp[0] = cv::Point(lines[j][0], lines[j][1]);
            ptTmp[1] = cv::Point(lines[j][2], lines[j][3]);
            ptTmp[2] = cv::Point(lines[i][0], lines[i][1]);
            ptTmp[3] = cv::Point(lines[i][2], lines[i][3]);

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

            if(x>20 && x < votosCantos.cols-20 && y > 20 && y <votosCantos.rows-20){
                cv::Point ip(x,y);
                                //cv::circle(roiDes,ip,1,cv::Scalar(255,255,0),2);
                votosCantos.at<double>(y,  x  ) += 2.f;
                votosCantos.at<double>(y+1,x+1) += 1.f;
                votosCantos.at<double>(y+1,x-1) += 1.f;
                votosCantos.at<double>(y-1,x-1) += 1.f;
                votosCantos.at<double>(y-1,x+1) += 1.f;
                votosCantos.at<double>(y  ,x+1) += 1.f;
                votosCantos.at<double>(y+1,x  ) += 1.f;
                votosCantos.at<double>(y-1,x  ) += 1.f;
                votosCantos.at<double>(y  ,x-1) += 1.f;
            }
        }

    cv::Mat filtroX = cv::Mat::zeros(votosCantos.cols,votosCantos.rows,CV_8UC1);
    cv::line(filtroX,cv::Point(0,0),cv::Point(filtroX.cols,filtroX.rows),cv::Scalar(255),50);
    cv::line(filtroX,cv::Point(filtroX.cols,0),cv::Point(0,filtroX.rows),cv::Scalar(255),50);

    cv::Mat cpyVotos;
    votosCantos.copyTo(cpyVotos,filtroX);

    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::minMaxLoc( cpyVotos, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
    cv::circle(roiDes,maxLoc,5,cv::Scalar(255,255,0),-1);
    cantoProximo = cv::Point(maxLoc.x+centroImg.x-deltaVan,maxLoc.y+centroImg.y-deltaVan);


//                cv::imshow("cpyVotos",cpyVotos);
//                cv::imshow("votosCantos",votosCantos);

//                cv::imshow("roiDes",roiDes);
//                cv::waitKey();


}

cv::Point2f Marcador::getCentroImg(){
    return cv::Point2f(centroImg.x,centroImg.y);
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



    for(int i =0;i<n;i++){
        marco.push_back(Marcador());
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
    CvPoint2D32f* ptmp = marco[0].CentroTab(posM);
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


int nmed = 0;
int  mensuriumAGE::AcharTabs(cv::Mat img, int n, int npl, cv::Mat imgDes){

    CvPoint2D32f p[n][3];

    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);

    cv::Mat imgC;
    img.copyTo(imgC);

    //std::cout<<"Ajustando Imagem para ser Analizada!"<<std::endl;

    cv::Mat cinza = cv::Mat(imgC.cols, imgC.rows, CV_8SC1);
    cv::Mat pb = cv::Mat(imgC.cols, imgC.rows, CV_8SC1);
    //cv::Mat hsv = cv::Mat(imgC.cols, imgC.rows, CV_8SC3);
    //cv::Mat pbHSV = cv::Mat(imgC.cols, imgC.rows, CV_8SC1);
    //cv::cvtColor(img,hsv,CV_BGR2HSV);
    //    cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(255,255,48),pbHSV);
    //    cv::threshold(pbHSV, pb,1,255 , CV_THRESH_BINARY_INV);// 3 = 70 =72

    //cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(255,100,50),pbHSV);
    //cv::threshold(pbHSV,pb,10,255 , CV_THRESH_BINARY_INV);// 3 = 70 =72

    cv::cvtColor(imgC, cinza, CV_BGR2GRAY);

    //cv::threshold(cinza, pb,70,255 , CV_THRESH_BINARY);// 3 = 70 =72
    //cv::threshold(pbHSV,pbHSV,5,255 , CV_THRESH_BINARY_INV);// 3 = 70 =72
    //    cv::imshow("cinzaPre",cinza);
    //    cv::waitKey(0);
    cv::equalizeHist( cinza, cinza );

    cv::adaptiveThreshold(cinza,pb,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,81,17);
    int nAchado = 0;


    //std::cout<<"Imagem para ser Analizada! Incianado busca de "<<n<<" Alvos!"<<std::endl;

    for(int i =0;i<n;i++){

        CvSize patternSize = cvSize(4, 4);//CvSize(6, 8);cel 6, 9
        std::vector<cv::Point2f> corners;
        //std::cout<<"Buscando Alvo: "<<nAchado+1<<std::endl;
        bool found = cv::findChessboardCorners( pb, patternSize, corners, CV_CALIB_CB_NORMALIZE_IMAGE);
        //std::cout<<"Fim da busca alvo: "<<nAchado+1<<std::endl;

        if(found){

            //std::cout<<"Alvo "<<nAchado+1<<" encontrado!"<<std::endl;
            cv::drawChessboardCorners( imgDes, patternSize, cv::Mat(corners), found );

            nAchado++;
            //     std::cout<<"c placa= "<<nAchado<<std::endl;
            //std::cout<<"Calculando pontos de interesse 2D!"<<std::endl;
            placa[npl].marco[i].calcCantosDigonal(corners); //CentroTab(cornerCount, corners);
            cv::circle(imgDes,placa[npl].marco[i].getCentroImg(),3,cv::Scalar(0,255,0),3);

            placa[npl].marco[i].getCentroImg();

            //placa[npl].marco[i].gettPontosTab3D(), pontosCam
            //std::cout<<"Calculando pontos de interesse 3D!"<<std::endl;
            cv::Mat Ponto3d = placa[npl].marco[i].PontosTab3D();
            //std::cout<<"Pontos tab 3D: "<<Ponto3d<<std::endl;
            //std::cout<<"Cantos img: "<<corners<<std::endl;
            //std::cout<<"Matriz Câmera: "<<cameraMatrix<<std::endl;
            //std::cout<<"Coef. Distorção: "<<distCoeffs<<std::endl;

            cv::cornerSubPix( cinza, corners, cv::Size(11,11),cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

            cv::solvePnP(Ponto3d, corners, cameraMatrix, distCoeffs, rvec, tvec, false);
            //std::cout<<"tvec "<<i<<tvec<<std::endl;

            placa[npl].marco[i].setPosicaoMONO( tvec);
            //std::cout<<"tMarco "<<i<<": "<<placa[npl].marco[i].getPosicaoMONO()<<std::endl;
            placa[npl].marco[i].setOrientacao(rvec);

            //std::cout<<"Calculando distancia total câmera - alvo!"<<std::endl;
            float dist = sqrt(tvec.at<double>(0,0)*tvec.at<double>(0,0)+tvec.at<double>(1,0)*tvec.at<double>(1,0)+tvec.at<double>(2,0)*tvec.at<double>(2,0));
            //std::cout<<"Desenhando na imagem de saida!"<<std::endl;
            if(!imgDes.empty()){
                cv::putText( imgDes,cv::format("Dist: %f",dist), cv::Point(placa[npl].marco[i].getCentroImg().x,placa[npl].marco[i].getCentroImg().y)+cv::Point(-100,-50), 1, 3,cv::Scalar(255,0,255),3);
//                cv::putText( imgDes,cv::format("Rot: %f,%f,%f",rvec.at<double>(0,0)*180.f/CV_PI,rvec.at<double>(1,0)*180.f/CV_PI,rvec.at<double>(2,0)*180.f/CV_PI),
//                             cv::Point(placa[npl].marco[i].getCentroImg().x,placa[npl].marco[i].getCentroImg().y)+cv::Point(-100,-100), 1, 3,cv::Scalar(255,0,255),3);

                cv::putText( imgDes,cv::format("Pos: %f,%f,%f",tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0)),
                             cv::Point(placa[npl].marco[i].getCentroImg().x,placa[npl].marco[i].getCentroImg().y)+cv::Point(-100,-100), 1, 3,cv::Scalar(255,0,255),3);
            }

           // std::cout<<"Rastreando cor!"<<std::endl;

            placa[npl].marco[i].VerificaCor(img,padCor,padCor,4);

            placa[npl].marco[i].AcharCantoProx(img,200,imgDes);

            placa[npl].marco[i].IdentQRCode(img,500);


            if(placa[npl].marco[i].getCor() == 0) cv::putText( imgDes,cv::format("Cor: %i : Amarelo",placa[npl].marco[i].getCor()), placa[npl].marco[i].getCentroImg(), 1, 3,cv::Scalar(0,255,255),3);
            if(placa[npl].marco[i].getCor() == 1) cv::putText( imgDes,cv::format("Cor: %i : Vermelho",placa[npl].marco[i].getCor()), placa[npl].marco[i].getCentroImg(), 1, 3,cv::Scalar(0,0,255),3);
            if(placa[npl].marco[i].getCor() == 2) cv::putText( imgDes,cv::format("Cor: %i : Verde",placa[npl].marco[i].getCor()), placa[npl].marco[i].getCentroImg(), 1, 3,cv::Scalar(0,255,0),3);
            if(placa[npl].marco[i].getCor() == 3) cv::putText( imgDes,cv::format("Cor: %i : Azul",placa[npl].marco[i].getCor()), placa[npl].marco[i].getCentroImg(), 1, 3,cv::Scalar(255,0,0),3);


            //std::cout<<"Eliminando alvo"<<nAchado<<"da imagem!"<<std::endl;
            cv::line(pb, placa[npl].marco[i].getCantosDigonal()[0], placa[npl].marco[i].getCantosDigonal()[1], cv::Scalar(0,0,0), 50, 1, 0);
            cv::line(pb, placa[npl].marco[i].getCantosDigonal()[2], placa[npl].marco[i].getCantosDigonal()[3], cv::Scalar(0,0,0), 50, 1, 0);

            p[i][0] = placa[npl].marco[i].getCentroImg();
            p[i][1] = placa[npl].marco[i].getCantosDigonal()[0];
            p[i][2] = placa[npl].marco[i].getCantosDigonal()[1];

            //std::cout<<"Fim do Processo para o alvo: "<<nAchado<<std::endl;


        }else{
            std::cout<<"Alvo"<<n<<"não encontrado!"<<std::endl;
        }

        //cv::imshow("AcharTab",pb);

    }

    try{


        if(nAchado == n){

            std::vector<Marcador> marcoTmp(placa[npl].marco);
            for(int io = 0;io<nAchado;io++){
                if(marcoTmp[io].getCor() == 0){
                    placa[npl].marco[0] = marcoTmp[io];
                }
                if(marcoTmp[io].getCor() == 1){
                    placa[npl].marco[1] = marcoTmp[io];
                }
                if(marcoTmp[io].getCor() == 2){
                    placa[npl].marco[2] = marcoTmp[io];
                }
                if(marcoTmp[io].getCor() == 3){
                    placa[npl].marco[3] = marcoTmp[io];
                }
            }
            std::cout<<"Medição:"<<nmed++<<std::endl;
            std::cout<<"tvec 0"<<placa[npl].marco[0].getPosicaoMONO()<<" cor: "<<placa[npl].marco[0].getCor()<<std::endl;
            std::cout<<"tvec 1"<<placa[npl].marco[1].getPosicaoMONO()<<" cor: "<<placa[npl].marco[1].getCor()<<std::endl;
            std::cout<<"tvec 2"<<placa[npl].marco[2].getPosicaoMONO()<<" cor: "<<placa[npl].marco[2].getCor()<<std::endl;
            std::cout<<"tvec 3"<<placa[npl].marco[3].getPosicaoMONO()<<" cor: "<<placa[npl].marco[3].getCor()<<std::endl;

            Alinhar(imgDes);
            Orientar(imgDes);
        }
    }catch(int e){
        std::cout << "An exception occurred. Exception Nr. " << e << '\n';
    }

    placa[npl].setnMarcoAch(nAchado);

    //std::cout<<"Fim do processo de rastreaneto dos alvos!"<<std::endl;

    return nAchado;
}


mensuriumAGE::mensuriumAGE()
    :method(BP)
    ,ndisp(64)
    ,fs(3)
{
    placa.push_back(Placa());
    placa.push_back(Placa());
    placa[0].Inic(4);
    placa[1].Inic(4);
    //    cv::FileStorage fs("MatCam.xml", cv::FileStorage::READ);
    //    fs["MatCam"]>>cameraMatrix;
    //    fs.release();
    //    cv::FileStorage fs2("MatCam.xml", cv::FileStorage::READ);
    //    fs2["Distortions"]>>distCoeffs;
    //    cv::FileStorage fsyml("/home/leandro/QtProjects/CamAGERoot/build-CamAGERoot-Desktop-Debug/calibracao_Basler.yml", cv::FileStorage::READ);
    //    fsyml["camera_matrix"]>>cameraMatrix;
    //    fsyml["distortion_coefficients"]>>distCoeffs;
    //    std::cout<<"Mensurium inicializado com Sucesso!"<<std::endl;
    cv::FileStorage fs("config/CalIV400.yml", cv::FileStorage::READ);//"config/calibracao_Basler.yml""config/CalIV400.yml"
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    try
    {
        std::cout << "Dispositivo OpenCL:" << cv::ocl::Context::getContext()->getDeviceInfo().deviceName << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cout << "Erro: " << e.what() << std::endl;
    }
}


Placa mensuriumAGE::getPlaca(int i){
    return placa[i];
}


void mensuriumAGE::AcharCentro1Tab(cv::Mat img, Marcador& marco, unsigned int largura, unsigned int altura, float tamanho)
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

        marco.setPosicaoMONO(trans);
        marco.setOrientacao(R);

        double dist = sqrt(pow(trans.at<double>(0,0),2)+pow(trans.at<double>(1,0),2)+pow(trans.at<double>(2,0),2));
        //std::cout<<"Dist: "<<dist<<std::endl;
        cv::Point temp = marco.getCentroImg();
        cv::putText( img,cv::format("Dist: %f",dist), temp, 1, 1,cv::Scalar(255,0,255));
        cv::circle(img,temp,5,cv::Scalar(255,0,255),3,2,0);
        cv::circle(img,cvPoint(img.cols/2,img.rows/2),5,cv::Scalar(0,0,255),3,2,0);
        cv::line(img,temp,cvPoint(img.cols/2,img.rows/2),cv::Scalar(0,0,0),3,2,0);
        marco.setValido();
    }
}


cv::Mat mensuriumAGE::Stereo(cv::Mat imgE, cv::Mat imgD){

    cv::cvtColor(imgE,imgE,CV_RGB2GRAY);
    cv::cvtColor(imgD,imgD,CV_RGB2GRAY);

    cv::Mat disp;
    cv::StereoBM sbm;
    //    sbm.state->SADWindowSize = 5;
    //    sbm.state->numberOfDisparities = 3*16;
    //    sbm.state->preFilterSize = 5;
    //    sbm.state->preFilterCap = 60;
    //    sbm.state->minDisparity = 0;
    //    sbm.state->textureThreshold = 100;
    //    sbm.state->uniquenessRatio = 10;
    //    sbm.state->speckleWindowSize = 255;
    //    sbm.state->speckleRange = 255;
    //    sbm.state->disp12MaxDiff = 255;

    sbm.state->preFilterCap = 31;
    sbm.state->SADWindowSize = 9;
    sbm.state->minDisparity = 0;
    sbm.state->numberOfDisparities = 128;
    sbm.state->textureThreshold = 10;
    sbm.state->uniquenessRatio = 15;
    sbm.state->speckleWindowSize = 100;
    sbm.state->speckleRange = 32;
    sbm.state->disp12MaxDiff = 1;

    sbm(imgE,imgD,disp);

    cv::Mat disp8U;
    double minVal, maxVal;
    minMaxLoc( disp, &minVal, &maxVal );
    disp.convertTo( disp8U, CV_8UC1, 255/(maxVal - minVal));

    cv::namedWindow("Disparidade",0);

    cv::imshow("Disparidade",disp8U);

    return disp;
}

void mensuriumAGE::StereoOCL(cv::Mat imgE, cv::Mat imgD){    
    cv::ocl::oclMat d_left, d_right;
    cv::cvtColor(imgE,imgE,CV_RGB2GRAY);
    cv::cvtColor(imgD,imgD,CV_RGB2GRAY);
    cv::resize(imgE, imgE, cv::Size(imgE.cols/fs, imgE.rows/fs));
    cv::resize(imgD, imgD, cv::Size(imgD.cols/fs, imgD.rows/fs));

    d_left.upload(imgE);
    d_right.upload(imgD);

    cv::imshow("Esquerda", imgE);
    cv::imshow("Direita", imgD);

    bp.ndisp = ndisp;
    csbp.ndisp = ndisp;

    cv::Mat disp;
    cv::ocl::oclMat d_disp;

    switch (method)
    {
    case BP:
        bp(d_left, d_right, d_disp);
        bp.estimateRecommendedParams(imgE.cols,imgE.rows,ndisp,bp.iters,bp.levels);
        break;
    case CSBP:
        csbp(d_left, d_right, d_disp);
        break;
    }

    // Show results
    d_disp.download(disp);
    disp.convertTo(disp, 0);
    //cv::putText(disp, cv::text(), Point(5, 25), FONT_HERSHEY_SIMPLEX, 1.0, Scalar::all(255));
    cv::imshow("disparity", disp);
    char key = cv::waitKey(3);

    switch (key)
    {
    case 'm':
    case 'M':
        switch (method)
        {
        case BP:
            method = CSBP;
            std::cout << "CSBP" << std::endl;
            break;
        case CSBP:
            method = BP;
            std::cout << "BP" << std::endl;
            break;
        }
        break;
    case '1':
        ndisp == 1 ? ndisp = 8 : ndisp += 8;
        std::cout << "ndisp: " << ndisp << std::endl;
        bp.ndisp = ndisp;
        csbp.ndisp = ndisp;
        break;
    case 'q':
    case 'Q':
        ndisp = std::max(ndisp - 8, 1);
        std::cout << "ndisp: " << ndisp << std::endl;
        bp.ndisp = ndisp;
        csbp.ndisp = ndisp;
        break;
    case '2':
        if (method == BP)
        {
            bp.iters += 1;
            std::cout << "iter_count: " << bp.iters << std::endl;
        }
        else if (method == CSBP)
        {
            csbp.iters += 1;
            std::cout << "iter_count: " << csbp.iters << std::endl;
        }
        break;
    case 'w':
    case 'W':
        if (method == BP)
        {
            bp.iters = std::max(bp.iters - 1, 1);
            std::cout << "iter_count: " << bp.iters << std::endl;
        }
        else if (method == CSBP)
        {
            csbp.iters = std::max(csbp.iters - 1, 1);
            std::cout << "iter_count: " << csbp.iters << std::endl;
        }
        break;
    case '3':
        if (method == BP)
        {
            bp.levels += 1;
            std::cout << "level_count: " << bp.levels << std::endl;
        }
        else if (method == CSBP)
        {
            csbp.levels += 1;
            std::cout << "level_count: " << csbp.levels << std::endl;
        }
        break;
    case 'e':
    case 'E':
        if (method == BP)
        {
            bp.levels = std::max(bp.levels - 1, 1);
            std::cout << "level_count: " << bp.levels << std::endl;
        }
        else if (method == CSBP)
        {
            csbp.levels = std::max(csbp.levels - 1, 1);
            std::cout << "level_count: " << csbp.levels << std::endl;
        }
        break;
    case '4':
        fs += 1;
        break;
    case 'r':
    case 'R':
        fs = std::max(fs - 1, 1);
        break;
    }
}

void mensuriumAGE::setarCores(cv::Scalar corI,cv::Scalar corF,int id){

    padCor[id][0] = corI;
    padCor[id][1] = corF;

}

void mensuriumAGE::Rodar(char* nomeJan, cv::Mat imgE, cv::Mat imgD){


    cv::Mat imgSaida;
    imgE.copyTo(imgSaida);

    AcharTabs(imgE,4,0,imgSaida);
    //std::cout<<"Ponto 0= "<<placa[0].marco->getPosicaoMONO()<<std::endl;

    if (placa[0].getnMarcoAch()==4){
        std::cout<<"Calculando centro do conjunto de alvos!"<<std::endl;
        placa[0].CalcentroPlaca();
        cv::circle(imgSaida,placa[0].getPosCentroImg(),5,cv::Scalar(255,0,255),2,1,0);

        //        for(int i= 0;i<4;i++ ){
        //                        placa[0].marco[i].AcharCantoProx(imgE,150,imgSaida);
        //        }


                        cv::line(imgSaida,placa[0].marco[0].getCantoProx(),placa[0].marco[1].getCantoProx(),cv::Scalar(0,0,255),3);
                        cv::line(imgSaida,placa[0].marco[1].getCantoProx(),placa[0].marco[3].getCantoProx(),cv::Scalar(0,0,255),3);
                        cv::line(imgSaida,placa[0].marco[3].getCantoProx(),placa[0].marco[2].getCantoProx(),cv::Scalar(0,0,255),3);
                        cv::line(imgSaida,placa[0].marco[2].getCantoProx(),placa[0].marco[0].getCantoProx(),cv::Scalar(0,0,255),3);


    }

    //    if(!imgD.empty()){
    //        AcharTabs(imgD,4,trans,1,imgSaida);
    //        placa[1].CalcentroPlaca();
    //        if(focoCalDim == 0) CalibrarFoco(imgE,imgD,cv::Size(6,9),3780.f,56.f);
    //        std::cout<<"Pontos R: "<<cv::Point(imgE.cols/2,-imgD.rows/2)<<" ; "<<std::endl;
    //        std::cout<<"Amarelo:"<<std::endl;
    //        placa[0].marco[0].setPosicaoStereo(XYZCamCaract(placa[0].marco[0].getCentroImg()-cv::Point(imgE.cols/2,-imgD.rows/2),placa[1].marco[0].getCentroImg()-cv::Point(imgE.cols/2,-imgD.rows/2),0,56.f));
    //        cv::circle(imgE,placa[0].marco[0].getCentroImg(),3,cv::Scalar(0,255,0),3);
    //        cv::circle(imgD,placa[1].marco[0].getCentroImg(),3,cv::Scalar(0,255,0),3);
    ////        std::cout<<"Pontos E: "<<placa[0].marco[0].getCentroImg()<<" ; "<<std::endl;
    ////        std::cout<<"Pontos D: "<<placa[1].marco[0].getCentroImg()<<" ; "<<std::endl;

    //        std::cout<<"Vermelho:"<<std::endl;
    //        placa[0].marco[1].setPosicaoStereo(XYZCamCaract(placa[0].marco[1].getCentroImg()-cv::Point(imgE.cols/2,-imgD.rows/2),placa[1].marco[1].getCentroImg()-cv::Point(imgE.cols/2,-imgD.rows/2),0,56.f));
    //        cv::circle(imgE,placa[0].marco[1].getCentroImg(),3,cv::Scalar(0,0,255),3);
    //        cv::circle(imgD,placa[1].marco[1].getCentroImg(),3,cv::Scalar(0,255,255),3);
    ////        std::cout<<"Pontos E: "<<placa[0].marco[1].getCentroImg()<<" ; "<<std::endl;
    ////        std::cout<<"Pontos D: "<<placa[1].marco[1].getCentroImg()<<" ; "<<std::endl;

    //        std::cout<<"Azul:"<<std::endl;
    //        placa[0].marco[2].setPosicaoStereo(XYZCamCaract(placa[0].marco[2].getCentroImg()-cv::Point(imgE.cols/2,-imgD.rows/2),placa[1].marco[2].getCentroImg()-cv::Point(imgE.cols,-imgD.rows/2),0,56.f));
    //        std::cout<<"Verde:"<<std::endl;
    //        placa[0].marco[3].setPosicaoStereo(XYZCamCaract(placa[0].marco[3].getCentroImg()-cv::Point(imgE.cols/2,-imgD.rows/2),placa[1].marco[3].getCentroImg()-cv::Point(imgE.cols,-imgD.rows/2),0,56.f));
    //        std::cout<<"Centro:"<<std::endl;
    //        XYZCamCaract(placa[0].getPosCentroImg(),placa[1].getPosCentroImg(),0,56.f);

    //        placa[1].marco[0].setPosicaoStereo(placa[0].marco[0].getPosicaoStereo());
    //        placa[1].marco[1].setPosicaoStereo(placa[0].marco[1].getPosicaoStereo());
    //        placa[1].marco[2].setPosicaoStereo(placa[0].marco[2].getPosicaoStereo());
    //        placa[1].marco[3].setPosicaoStereo(placa[0].marco[3].getPosicaoStereo());
    //    }

    cv::resize(imgSaida,imgSaida,cv::Size(imgSaida.cols/4,imgSaida.rows/4));

    cv::imshow(nomeJan,imgSaida);

    //std::cout<<"Fim RODAR!"<<std::endl;
}

float mensuriumAGE::CalibrarFoco(cv::Mat imgE, cv::Mat imgD, cv::Size tamTabRef, float distZ, float b_mm){

    Marcador marcoRefE,marcoRefD;
    AcharCentro1Tab(imgE, marcoRefE, tamTabRef.width, tamTabRef.height);
    AcharCentro1Tab(imgD, marcoRefD, tamTabRef.width, tamTabRef.height);

    double dispCentros = marcoRefE.getCentroImg().x-marcoRefD.getCentroImg().x;

    focoCalDim = (distZ*dispCentros / b_mm);

    std::cout<<"Disparidade: "<<dispCentros<<std::endl;
    std::cout<<"Foco em pixels: "<<focoCalDim<<std::endl;

    XYZCamCaract(marcoRefE.getCentroImg(),marcoRefD.getCentroImg());

    return focoCalDim;
}

cv::Point3d mensuriumAGE::XYZCamCaract(cv::Point ptE, cv::Point ptD, float fx_pixel, float b_mm){
    float disp_pixels = ptE.x - ptD.x;
    //    std::cout<<"Pontos: "<<ptE<<" ; "<<ptD<<std::endl;
    //    std::cout<<"Disparidade: "<< disp_pixels<<std::endl;
    //    std::cout<<"b: "<< b_mm<<std::endl;
    double X = (b_mm * (ptE.x + ptD.x)) / (2*disp_pixels);
    double Y = (b_mm * (ptE.y + ptD.y)) / (2*disp_pixels);
    double Z = 0;
    if(fx_pixel != 0){
        Z = b_mm * fx_pixel / disp_pixels;
        //        std::cout<<"fp: "<< fx_pixel<<std::endl;
    }
    else{
        if(focoCalDim != 0){
            //            std::cout<<"fp: "<< focoCalDim<<std::endl;
            Z = b_mm * focoCalDim / disp_pixels;}
    }

    double dist = sqrt(pow(X,2)+pow(Y,2)+pow(Z,2));
    std::cout<<"Ponto 3D: "<<cv::Point3d(X,Y,Z)<<std::endl;
    //    std::cout<<"dist 3D: "<<dist<<std::endl;
    return(cv::Point3d(X,Y,Z));
}


void mensuriumAGE::Alinhar(cv::Mat imgDes){

    cv::line(imgDes,placa[0].marco[2].getCentroImg(),placa[0].marco[3].getCentroImg(),cv::Scalar(125,125,0),3);
    cv::line(imgDes,placa[0].marco[1].getCentroImg(),placa[0].marco[0].getCentroImg(),cv::Scalar(0,125,125),3);

    cv::Mat tveci = placa[0].marco[2].getPosicaoMONO();
    //std::cout<<"tveci"<<tveci<<std::endl;
    //float disti = sqrt(tveci.at<double>(0,0)*tveci.at<double>(0,0)+tveci.at<double>(1,0)*tveci.at<double>(1,0)+tveci.at<double>(2,0)*tveci.at<double>(2,0));
    cv::Mat tvecf = placa[0].marco[3].getPosicaoMONO();
    //std::cout<<"tvecf"<<tvecf<<std::endl;
    //float distf = sqrt(tvecf.at<double>(0,0)*tvecf.at<double>(0,0)+tvecf.at<double>(1,0)*tvecf.at<double>(1,0)+tvecf.at<double>(2,0)*tvecf.at<double>(2,0));

    //    deltaXYZ[0] =placa[0].marco[0].getCentroImg().x-placa[0].marco[3].getCentroImg().y;
    //    deltaXYZ[1] =placa[0].marco[0].getCentroImg().x-placa[0].marco[3].getCentroImg().y;

    deltaXYZ[0] =tvecf.at<double>(0,0)-tveci.at<double>(0,0);
    deltaXYZ[1] =tvecf.at<double>(1,0)-tveci.at<double>(1,0);
    deltaXYZ[2] =tvecf.at<double>(2,0)-tveci.at<double>(2,0);


    std::cout<<"Delta POS: ("<<deltaXYZ[0]<<","<<deltaXYZ[1]<<","<<deltaXYZ[2]<<")"<<std::endl;


}

float mensuriumAGE::calcAng(cv::Point2f ptCG, cv::Point2f ptCe){
    double r = sqrt((ptCG.x-ptCe.x)*(ptCG.x-ptCe.x)+(ptCG.y-ptCe.y)*(ptCG.y-ptCe.y));
    //std::cout<<"r = "<<r;
    float projSin = (ptCe.y - ptCG.y)/r;
    //std::cout<<" projSin = "<<projSin;
    float ang = asin(projSin);
    //std::cout<<" asin = "<<ang;
    ang = 180*ang/CV_PI;
    //std::cout<<" ang = "<<ang<<std::endl;
    return ang;
}

void mensuriumAGE::Orientar(cv::Mat imgDes){

    cv::line(imgDes,placa[0].marco[0].getCentroImg(),placa[0].marco[2].getCentroImg(),cv::Scalar(0,0,255),3);
    cv::line(imgDes,placa[0].marco[1].getCentroImg(),placa[0].marco[3].getCentroImg(),cv::Scalar(0,0,255),3);

    cv::Mat tveci = placa[0].marco[0].getPosicaoMONO();
    //std::cout<<"tveci"<<tveci<<std::endl;

    cv::Mat tvecf = placa[0].marco[2].getPosicaoMONO();
    //std::cout<<"tvecf"<<tvecf<<std::endl;

    cv::Point2f ptCG(tvecf.at<double>(0,0),tvecf.at<double>(1,0));
    cv::Point2f ptCe(tveci.at<double>(0,0),tveci.at<double>(1,0));

    deltaABC[2] = calcAng(ptCG,ptCe);

    ptCG = cv::Point2f(tvecf.at<double>(0,0),tvecf.at<double>(2,0));
    ptCe = cv::Point2f(tveci.at<double>(0,0),tveci.at<double>(2,0));

    deltaABC[1] = calcAng(ptCG,ptCe);

    ptCG = cv::Point2f(tvecf.at<double>(1,0),tvecf.at<double>(2,0));
    ptCe = cv::Point2f(tveci.at<double>(1,0),tveci.at<double>(2,0));

    deltaABC[0] = calcAng(ptCG,ptCe);

    //std::cout<<"ANGs[0-2]: ("<<deltaABC[0]<<","<<deltaABC[1]<<","<<deltaABC[2]<<")"<<std::endl;

    tveci = placa[0].marco[1].getPosicaoMONO();
    //std::cout<<"tveci"<<tveci<<std::endl;

    tvecf = placa[0].marco[3].getPosicaoMONO();
    //std::cout<<"tvecf"<<tvecf<<std::endl;

    float ang2[3];

    ptCG = cv::Point2f(tvecf.at<double>(0,0),tvecf.at<double>(1,0));
    ptCe = cv::Point2f(tveci.at<double>(0,0),tveci.at<double>(1,0));

    ang2[2] = calcAng(ptCG,ptCe);
    deltaABC[2] = ang2[2] - deltaABC[2];


    ptCG = cv::Point2f(tvecf.at<double>(0,0),tvecf.at<double>(2,0));
    ptCe = cv::Point2f(tveci.at<double>(0,0),tveci.at<double>(2,0));

    ang2[1] = calcAng(ptCG,ptCe);
    deltaABC[1] = ang2[1] - deltaABC[1];

    ptCG = cv::Point2f(tvecf.at<double>(1,0),tvecf.at<double>(2,0));
    ptCe = cv::Point2f(tveci.at<double>(1,0),tveci.at<double>(2,0));

    ang2[0] = calcAng(ptCG,ptCe);
    deltaABC[0] = ang2[0] - deltaABC[0];

    //std::cout<<"ANGs [1-3]: ("<<ang2[0]<<","<<ang2[1]<<","<<ang2[2]<<")"<<std::endl;

    std::cout<<"ANGs delta: ("<<deltaABC[0]<<","<<deltaABC[1]<<","<<deltaABC[2]<<")"<<std::endl;

}

void mensuriumAGE::getDeltaXYZ(float *&dXYZ){
    dXYZ[0] = deltaXYZ[0];
    dXYZ[1] = deltaXYZ[1];
    dXYZ[2] = deltaXYZ[2];
}

void mensuriumAGE::getDeltaABC(float *&dABC){

    dABC[0] = deltaABC[0];
    dABC[1] = deltaABC[1];
    dABC[2] = deltaABC[2];

}
