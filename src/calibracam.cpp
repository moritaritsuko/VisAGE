#include "VISAGE/calibracam.hpp"
#include "opencv/cv.h"
#include "opencv2/opencv.hpp"
#include <pylon/PylonIncludes.h>

CalibraCam::CalibraCam()
{
}

using namespace cv;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
    corners.resize(0);

    switch(patternType)
    {
    case CHESSBOARD:
    case CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

    case ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                          float(i*squareSize), 0));
        break;

    default:
        CV_Error(CV_StsBadArg, "Unknown pattern type\n");
    }
}

static bool runCalibration( vector<vector<Point2f> > imagePoints,
                            Size imageSize, Size boardSize, Pattern patternType,
                            float squareSize, float aspectRatio,
                            int flags, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,
                            double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);


    return ok;
}

static void saveCameraParams( const string& filename,
                              Size imageSize, Size boardSize,
                              float squareSize, float aspectRatio, int flags,
                              const Mat& cameraMatrix, const Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs,
                              const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
                 flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                 flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                 flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                 flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());

        for( int i = 0; i < (int)rvecs.size(); i++ )
        {

            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }

        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }


}

static bool runAndSave(const string& outputFilename,
                       const vector<vector<Point2f> >& imagePoints,
                       Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                       float aspectRatio, int flags, Mat& cameraMatrix,
                       Mat& distCoeffs, bool writeExtrinsics, bool writePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                             aspectRatio, flags, cameraMatrix, distCoeffs,
                             rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if( ok )
        saveCameraParams( outputFilename, imageSize,
                          boardSize, squareSize, aspectRatio,
                          flags, cameraMatrix, distCoeffs,
                          writeExtrinsics ? rvecs : vector<Mat>(),
                          writeExtrinsics ? tvecs : vector<Mat>(),
                          writeExtrinsics ? reprojErrs : vector<float>(),
                          writePoints ? imagePoints : vector<vector<Point2f> >(),
                          totalAvgErr );

    FileStorage fsC("MatCam.xml", FileStorage::WRITE);
    fsC<<"MatCam"<< cameraMatrix;
    FileStorage fsR("Rotacao.xml", FileStorage::WRITE);
    fsR<<"Rotacao"<< rvecs;
    FileStorage fsT("Translacao.xml", FileStorage::WRITE);
    fsT<<"Translacao"<< tvecs;

    fsC.release();
    fsR.release();
    fsT.release();


    return ok;
}

void CalibraCam::Calibrar(Size boardSize,float tamQuad,bool tipoCam,Pylon::CInstantCamera &camera){

    Size imageSize;
    float squareSize = 1.f, aspectRatio = 1.f;
    Mat cameraMatrix, distCoeffs;
    const char* outputFilename = "out_camera_data.yml";

    int i, nframes = 5;
    bool writeExtrinsics = false, writePoints = false;
    bool undistortImage = false;
    int flags = 0;
    VideoCapture capture;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    int cameraId = 0;
    vector<vector<Point2f> > imagePoints;
    Pattern pattern = CHESSBOARD;
    Pylon::PylonAutoInitTerm autoInitTerm;
    //Pylon::CInstantCamera camera;
    Pylon::CGrabResultPtr ptrGrabResult;
    bool capOpen = false;

    //Parametros de configuracao
    //boardSize.width = 9;
    //boardSize.height = 6;
    squareSize = tamQuad;
    writePoints = true;
    writeExtrinsics = true;

    if(!tipoCam){
        capture.open(cameraId);

        if( !capture.isOpened() )
            printf( "Could not initialize video (%d) capture\n",cameraId );
    }else{
        //camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        camera.StartGrabbing();
    }

    namedWindow( "Calibracao", 1 );

    for(i = 0;;i++)
    {
        Mat view, viewGray;
        bool blink = false;

        if( capture.isOpened() )
        {
            capOpen = true;
            Mat view0;
            capture >> view0;
            view0.copyTo(view);
        }else{
novaimg:
            if(camera.IsGrabbing()){
                capOpen = true;
                camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
                if (ptrGrabResult->GrabSucceeded()){
                    view = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, ptrGrabResult->GetBuffer());
                    cv::cvtColor(view,view,CV_BayerGB2RGB);
                }
            }else{capOpen = false;}
        }

        imageSize = view.size();

        vector<Point2f> pointbuf;
        viewGray = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1);
        cvtColor(view, viewGray, CV_BGR2GRAY);



        bool found;
        if(mode == CAPTURING){
            if(camera.IsGrabbing()){
comeco:
                cv::Mat cinzaMenor;
                cv::resize(viewGray,cinzaMenor,cv::Size(viewGray.cols/3,viewGray.rows/3));
                cv::Mat imgThreshM=cv::Mat(cinzaMenor.rows, cinzaMenor.cols, CV_8UC1);
                cv::threshold(cinzaMenor, imgThreshM, 20,255 , CV_THRESH_BINARY+CV_THRESH_OTSU);
                found = findChessboardCorners( cinzaMenor, boardSize, pointbuf,
                                               CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);
                if(found){
                    putText( cinzaMenor, "Aceitar Imagem? s ou n", cv::Point(cinzaMenor.rows/2-50,cinzaMenor.cols/2), 1, 1,Scalar(0,255,0));
                    imshow("imagem para Calibraçao", cinzaMenor);
                    int key = waitKey();
                    if(key == 1048691){
                        cv::Mat imgThresh=cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1);
                        cv::threshold(viewGray, imgThresh, 20,255 , CV_THRESH_BINARY+CV_THRESH_OTSU);
                        found = findChessboardCorners( imgThresh, boardSize, pointbuf,
                                                       CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);
                        if(found){
                            goto calibrar;
                        }else{
                            std::cout<<"Imagem Invalida"<<std::endl;
                        }
                    }else{
                        if(key == 1048686){
                            std::cout<<"Nova imagem!"<<std::endl;
                            goto novaimg;
                        }else{
                            std::cout<<"Tecla Invalida! key= "<<key<<std::endl;
                            goto comeco;
                        }
                    }
                }
            }else{
                found = findChessboardCorners( view, boardSize, pointbuf,
                                               CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);
            }
        }
calibrar:

        // improve the found corners' coordinate accuracy
        if(found) cornerSubPix( viewGray, pointbuf, Size(11,11),
                                Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

        if( mode == CAPTURING && found &&
                (!capOpen || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) )
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capOpen;
        }

        if(found)
            drawChessboardCorners( view, boardSize, Mat(pointbuf), found );


        string msg = mode == CAPTURING ? "100/100" :
                                         mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(undistortImage)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), nframes );
        }



        if( blink )
            bitwise_not(view, view);

        if( mode == CALIBRATED && undistortImage )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        if(capOpen){
            cv::Mat imgMMat;
            cv::resize(view,imgMMat,cvSize(view.cols/3,view.rows/3));
            putText( imgMMat, msg, cv::Point(50,50), 1, 1,
                     mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));
            imshow("Calibracao", imgMMat);
        }else{
            putText( view, msg, textOrigin, 1, 1,
                     mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));
            imshow("Calibracao", view);
        }


        int key = 0xff & waitKey(capOpen ? 50 : 500);

        if( (key & 255) == 27 )
            break;

        if( key == 'u' && mode == CALIBRATED )
            undistortImage = !undistortImage;

        if( capOpen && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }

        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
            if( runAndSave(outputFilename, imagePoints, imageSize,
                           boardSize, pattern, squareSize, aspectRatio,
                           flags, cameraMatrix, distCoeffs,
                           writeExtrinsics, writePoints))
                mode = CALIBRATED;
            else
                mode = DETECTION;
            if( !capOpen )
                break;
        }
        if(key == 's')break;

        goto novaimg;
    }

    camera.StopGrabbing();
    camera.Close();

    capture.release();
    cvDestroyWindow("Calibracao");


}


void CalibraCam::Calibrar(Size boardSize, float tamQuad,  Mat *imgCamera, int n, std::string nomeArq){


    int flags = 0;



    Pattern pattern = CHESSBOARD;

    Size imageSize;
    float aspectRatio = 1.f;
    Mat cameraMatrix, distCoeffs;
    const char* outputFilename = nomeArq.data();

    int i;
    bool writeExtrinsics = false, writePoints = false;

    vector<vector<Point2f> > imagePoints;


    //Parametros de configuracao
    float squareSize = tamQuad;
    writePoints = true;
    writeExtrinsics = true;

    namedWindow( "Calibracao", 1 );

    for(i = 0;i<n;i++)
    {
        Mat view, viewGray;

        imgCamera[i].copyTo(view);

        imageSize = view.size();

        vector<Point2f> pointbuf;

        cvtColor(view, viewGray, CV_BGR2GRAY);


        bool found = findChessboardCorners( viewGray, boardSize, pointbuf,CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);

        if(found){
            cornerSubPix( viewGray, pointbuf, Size(11,11),Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
            imagePoints.push_back(pointbuf);
        }else{
         std::cout<<"Processo abortado! Tabuleiro não encontrado na imagem: "<<i<<std::endl;
         break;
        }




    }

    runAndSave(outputFilename, imagePoints, imageSize,
                               boardSize, pattern, squareSize, aspectRatio,
                               flags, cameraMatrix, distCoeffs,
                               writeExtrinsics, writePoints);



}

void CalibraCam::CalibrarPorRegiao(Size boardSize, float tamQuad, bool tipoCam, Pylon::CInstantCamera &camera,char * nomeFile,int fs, bool autoRobo){

    cv::Mat imgCam;
    cv::Mat imgGray;
    cv::Mat copy;
    Pattern pattern = CHESSBOARD;
    cv::Mat cameraMatrix, distCoeffs;
    const char* outputFilename = nomeFile;

    float squareSize = tamQuad;
    float aspectRatio = 1.f;

    bool writeExtrinsics = true, writePoints = true;

    vector<vector<Point2f> > imagePoints;
    vector<Point2f> pointbuf;

    cv::VideoCapture cap;
    Pylon::PylonAutoInitTerm autoInitTerm;
    Pylon::CGrabResultPtr ptrGrabResult;

    if(tipoCam){
        camera.StartGrabbing();
    }else{
        cv::VideoCapture captmp (1);
        cap = captmp;
        captmp.release();
    }

    if(autoRobo){

    }



novaImg:


    if(camera.IsGrabbing() && tipoCam){
        camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        if (ptrGrabResult->GrabSucceeded()){
            imgCam = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, ptrGrabResult->GetBuffer());
            cv::cvtColor(imgCam,imgCam,CV_BayerGB2RGB);
        }
    }else{
        cap>>imgCam;
        if( imgCam.empty()) std::cout<<"Erro na Captura!"<<std::endl;
    }


    cv::Mat cinzaMenor;
    cv::cvtColor(imgCam,imgGray,CV_RGB2GRAY);
    if(tipoCam) {cv::resize(imgGray,cinzaMenor,cv::Size(imgGray.cols/fs,imgGray.rows/fs));}else{cinzaMenor = imgGray;}
    cv::Mat imgThreshM=cv::Mat(cinzaMenor.rows, cinzaMenor.cols, CV_8UC1);
    cv::adaptiveThreshold(cinzaMenor,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
    int found = findChessboardCorners( imgThreshM, boardSize, pointbuf,
                                       CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);

    if(found){
        cinzaMenor.copyTo(copy);
        drawChessboardCorners( copy, boardSize, Mat(pointbuf), found );
        cv::imshow("Aceitar Img? s, n Calibrar = c",copy);

        char key = cv::waitKey(0);

        if(key == 's'){

            //            if(camera.IsGrabbing()){
            //                camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            //                 if (ptrGrabResult->GrabSucceeded()){
            //                     imgCam = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, ptrGrabResult->GetBuffer());
            //                     cv::cvtColor(imgCam,imgCam,CV_BayerGB2RGB);
            //                 }
            //            }

            cv::Mat imgThreshM=cv::Mat(imgGray.rows, imgGray.cols, CV_8UC1);
            cv::adaptiveThreshold(imgGray,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
            cv::imshow("imgThreshM",imgThreshM);
            int foundF = findChessboardCorners( imgThreshM, boardSize, pointbuf,
                                                CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);
            if(foundF){
                cv::cvtColor(imgCam,imgGray,CV_RGB2GRAY);
                cornerSubPix( imgGray, pointbuf, Size(11,11),Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                imagePoints.push_back(pointbuf);
                std::cout<<"Postos Aceitos! k = "<<key<<std::endl;
            }else{
                std::cout<<"Postos Não encontrados nessa resolução! k = "<<key<<std::endl;
            }
            goto novaImg;

        }

        if(key == 'n'){
            std::cout<<"Pontos não Aceitos! k = "<<key<<std::endl;
            goto novaImg;
        }

        if(key == 'c'){
            goto calibre;
        }

        /*
        !!!!Codigo de movimetação do robo para nova posição!!!!!
        Verificar connec robo
        */
        if(autoRobo){

        }
        goto novaImg;

    }else{
        goto novaImg;
    }

calibre:
    int flags = 0;

    runAndSave(outputFilename, imagePoints, cv::Size(cinzaMenor.cols,cinzaMenor.rows),
               boardSize, pattern, squareSize, aspectRatio,
               flags, cameraMatrix, distCoeffs,
               writeExtrinsics, writePoints);

    std::cout<<"Calibrando!"<<std::endl;

}


