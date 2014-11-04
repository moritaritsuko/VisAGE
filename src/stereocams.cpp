#include <VISAGE/stereocams.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <unistd.h>
#include <iostream>

#include <QMessageBox>

StereoCameras::StereoCameras()
    : mStereoPhotoPtr(new StereoCameras::StereoPhoto)
    , mAutoInitTerm()
    , mTransportLayerFactory(Pylon::CTlFactory::GetInstance())
    , mDevices()
    , mCameras(2u)
    , mCameraNames()
    , mDisplayCapture(false)
    , mDisplayCaptureMutex()
{
    if (attachDevices())
    {
        // Triggers Configuration Event (CameraConfiguration.cpp)
        mCameras.Open();
        startDisplayCapture();
    }
    else
        std::cout << "AVISO: Câmeras Estéreo não foram encontradas." << std::endl;
}

void StereoCameras::exec()
{
    if (mCameras.IsOpen())
    {
        mDisplayCapture = true;
        auto stereoPhoto = mStereoPhotoPtr.get();

        registerCameraCapture(stereoPhoto);

        // Cameras Synchronization: Round-Robin Strategy
        mCameras.StartGrabbing(Pylon::GrabStrategy_UpcomingImage);
    }
}

StereoCameras::StereoPhoto* StereoCameras::getStereoPhotoPair()
{
    return mStereoPhotoPtr.get();
}

bool StereoCameras::attachDevices()
{
    if (mTransportLayerFactory.EnumerateDevices(mDevices) == 0)
        return false;

    int attached = 0;
    for (size_t i = 0; i < mDevices.size(); ++i)
    {
        std::string cameraName;
        // Attach device to Pylon's camera array
        auto device = mTransportLayerFactory.CreateDevice(mDevices[i]);
        auto ip = device->GetDeviceInfo().GetFullName();
        auto ip1 = "169.254.8.100";
        auto ip2 = "169.254.8.102";

        if (ip.find(ip1) != std::string::npos || ip.find(ip2) != std::string::npos)
        {
            mCameras[attached].Attach(device);
            Pylon::CInstantCamera &camera = mCameras[attached];

            // Define Camera Name for logging and OpenCV NamedWindow
            cameraName = camera.GetDeviceInfo().GetFullName();
            mCameraNames.push_back(cameraName);
            // Register Camera's Configuration
            camera.RegisterConfiguration
                    (
                        new CameraConfiguration("config/default_linux.pfs", 8192, 4096 * (int) (attached + 1), cameraName),
                        Pylon::RegistrationMode_ReplaceAll,
                        Pylon::Cleanup_Delete
                        );
            ++attached;
        }
    }
    return true;
}

void StereoCameras::registerCameraCapture(StereoCameras::StereoPhoto* stereoPhotoPtr)
{
    for (size_t i = 0; i < 2; ++i)
    {
        stereoPhotoPtr->cameras[i] = mCameraNames[i];
        mCameras[i].RegisterImageEventHandler
                (
                    new CameraCapture(mCameraNames[i], stereoPhotoPtr),
                    Pylon::RegistrationMode_ReplaceAll,
                    Pylon::Cleanup_Delete
                    );
    }
}

void StereoCameras::capture()
{
    Pylon::CGrabResultPtr grabResultPtr;

    if (mCameras.IsGrabbing())
    {
        // Triggers Capture Event
        mCameras.RetrieveResult(5000, grabResultPtr, Pylon::TimeoutHandling_ThrowException);
    }
}

CameraCapture::CameraCapture(std::string cameraName, StereoCameras::StereoPhoto* stereoPhotoPtr)
    : mCameraName(cameraName)
    , mCalibrationMatrices()
    , mCalibrationMatricesFiles({
                                "config/Q.xml",
                                "config/mx1.xml",
                                "config/my1.xml",
                                "config/mx2.xml",
                                "config/my2.xml"
                                })
    , mCalibrationMatricesNames({"Q", "mx1", "my1", "mx2", "my2"})
    , mPatternSize()
    , mStereoPhotoPtr(stereoPhotoPtr)
    , mThreshold(0.f)
{
    // Load Calibration Matrices from the XML files
    for (int i = 0; i < mCalibrationMatrices.size(); ++i)
    {
        cv::FileStorage fs(mCalibrationMatricesFiles[i], cv::FileStorage::READ);
        fs[mCalibrationMatricesNames[i]] >> mCalibrationMatrices[i];
        fs.release();
    }

    mPatternSize.width = 9;
    mPatternSize.height = 6;
}

void CameraCapture::OnImageGrabbed(Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& grabResultPtr)
{
    if (grabResultPtr->GrabSucceeded())
    {
        // OpenCV image CV_8U: 8-bits, 1 channel
        auto imageCamera = cv::Mat(grabResultPtr->GetHeight(), grabResultPtr->GetWidth(), CV_8UC1, grabResultPtr->GetBuffer());
        auto image = cv::Mat(grabResultPtr->GetHeight(), grabResultPtr->GetWidth(), CV_8UC1);
        auto cameraContextValue = grabResultPtr->GetCameraContext();

        cv::cvtColor(imageCamera, imageCamera, CV_BayerGB2RGB);

        // Left Camera
        if (cameraContextValue == 0)
        {
            mStereoPhotoPtr->matPair.first = imageCamera;
        }
        // Right Camera
        else
        {
            mStereoPhotoPtr->matPair.second = imageCamera;
        }
    }
    else
    {
        std::cout << mCameraName << "CameraCapture::OnImageGrabbed() ERROR: " << grabResultPtr->GetErrorCode() << " " << grabResultPtr->GetErrorDescription() << std::endl;
    }
}

double StereoCameras::CalibrarStCam(float tamQuad,cv::Size TamTab){

    exec();

    int fs = 3;

    cv::Mat imgD;
    cv::Mat imgE;
    cv::Mat copiaImgD;
    cv::Mat copiaImgE;
    std::vector<std::vector<cv::Point2f> > imagePointsD;
    std::vector<std::vector<cv::Point2f> > imagePointsE;
    std::vector<cv::Point2f> pointbufD;
    std::vector<cv::Point2f> pointbufE;

    float squareSize = tamQuad;

    QMessageBox msgBoxint;
    msgBoxint.setText("Calibração iniciada!");
    msgBoxint.exec();

novaImg:
    mDisplayCaptureMutex.lock();
    capture();
    mDisplayCaptureMutex.unlock();
    auto photoPair = getStereoPhotoPair();
    auto photo1 = photoPair->matPair.first;
    auto photo2 = photoPair->matPair.second;
    cv::waitKey(10);

    if (!photo2.empty())
    {
        photo1.copyTo(imgE);

        cv::Mat imgGrayE;
        cv::Mat imgGrayD;
        cv::Mat cinzaMenor;
        cv::cvtColor(imgE,imgGrayE,CV_RGB2GRAY);
        cv::resize(imgGrayE,cinzaMenor,cv::Size(imgGrayE.cols/fs,imgGrayE.rows/fs));
        //        cv::Mat imgThreshM=cv::Mat(cinzaMenor.rows, cinzaMenor.cols, CV_8UC1);
        //        cv::adaptiveThreshold(cinzaMenor,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
        cv::imshow("cinzaMenor",cinzaMenor);
        int found1 = findChessboardCorners( cinzaMenor, TamTab, pointbufE,
                                            CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);

        cinzaMenor.copyTo(copiaImgE);

        photo2.copyTo(imgD);
        cv::cvtColor(imgD,imgGrayD,CV_RGB2GRAY);
        cv::resize(imgGrayD,cinzaMenor,cv::Size(imgGrayD.cols/fs,imgGrayD.rows/fs));
        //        imgThreshM=cv::Mat(cinzaMenor.rows, cinzaMenor.cols, CV_8UC1);
        //        cv::adaptiveThreshold(cinzaMenor,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
        int found2 = findChessboardCorners( cinzaMenor, TamTab, pointbufD,
                                            CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);
        cinzaMenor.copyTo(copiaImgD);

        //cv::imshow("testeD",copiaImgD);
        //cv::imshow("testeE",copiaImgE);
        if(found1 && found2){


            cv::drawChessboardCorners( copiaImgE, TamTab, cv::Mat(pointbufE), found1 );
            cv::drawChessboardCorners( copiaImgD, TamTab, cv::Mat(pointbufD), found2 );

            cv::imshow("testeD",copiaImgD);
            cv::imshow("testeE",copiaImgE);

            QMessageBox msgBox;
            msgBox.setText("Pontos encontrados!");
            msgBox.setInformativeText("Salvar pontos para Calibração?");
            msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard);
            msgBox.setDefaultButton(QMessageBox::Save);
            int ret = msgBox.exec();

            if(ret == QMessageBox::Save){
                //                imgThreshM=cv::Mat(imgGrayE.rows, imgGrayE.cols, CV_8UC1);

                //                cv::adaptiveThreshold(imgGrayE,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
                //                cv::imshow("imgThreshME",imgThreshM);
                int foundE = findChessboardCorners( imgGrayE, TamTab, pointbufE,
                                                    CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);

                //                cv::adaptiveThreshold(imgGrayD,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
                //                cv::imshow("imgThreshMD",imgThreshM);
                int foundD = findChessboardCorners( imgGrayD, TamTab, pointbufD,
                                                    CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);
                if(foundE && foundD){
                    cv::cvtColor(imgE,imgGrayE,CV_RGB2GRAY);
                    cv::cornerSubPix( imgGrayE, pointbufE, cv::Size(11,11),cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                    imagePointsE.push_back(pointbufE);

                    cv::cvtColor(imgD,imgGrayD,CV_RGB2GRAY);
                    cv::cornerSubPix( imgGrayD, pointbufD, cv::Size(11,11),cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                    imagePointsD.push_back(pointbufD);

                    QMessageBox msgBoxOK;
                    msgBoxOK.setText("Pontos Aceitos!");
                    msgBoxOK.exec();
                    goto pgtCalibrar;
                }else{
                    QMessageBox msgBoxOK;
                    msgBoxOK.setText("Postos não encontrados nessa resolução!");
                    msgBoxOK.exec();
                }
                goto novaImg;
            }else goto novaImg;

pgtCalibrar:

            QMessageBox msgBoxCal;
            msgBoxCal.setText("Pontos Salvos!");
            msgBoxCal.setInformativeText("Calibrar?");
            msgBoxCal.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
            msgBoxCal.setDefaultButton(QMessageBox::No);
            int respCal = msgBoxCal.exec();

            if(respCal == QMessageBox::Yes){
                goto calibrar;
            }else goto novaImg;


        }else{
            //            QMessageBox msgBoxOK;
            //            msgBoxOK.setText("Tabs não encontrados!");
            //            msgBoxOK.exec();
            goto novaImg;
        }

calibrar:

        std::vector<cv::Point3f> obj;
        obj.resize(0);

        for( int i = 0; i < TamTab.height; i++ )
            for( int j = 0; j < TamTab.width; j++ )
                obj.push_back(cv::Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));


        std::vector<std::vector<cv::Point3f> > objectPoints;
        for (int i = 0; i < imagePointsE.size(); ++i)
            objectPoints.push_back(obj);
        double RMS = cv::stereoCalibrate(objectPoints, imagePointsE, imagePointsD,
                                         CM1, D1, CM2, D2, imgD.size(), R, T, E, F,
                                         cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                         CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

        cv::stereoRectify(CM1, D1, CM2, D2, imgD.size(), R, T, R1, R2, P1, P2, Q);

        cv::FileStorage fs1("StereoBasler.yml", cv::FileStorage::WRITE);
        fs1 << "CM1" << CM1;
        fs1 << "CM2" << CM2;
        fs1 << "D1" << D1;
        fs1 << "D2" << D2;
        fs1 << "R" << R;
        fs1 << "T" << T;
        fs1 << "E" << E;
        fs1 << "F" << F;
        fs1 << "R1" << R1;
        fs1 << "R2" << R2;
        fs1 << "P1" << P1;
        fs1 << "P2" << P2;
        fs1 << "Q" << Q;

        fs1.release();

        QMessageBox msgBoxOK;
        std::string msg("Calibração concluída, RMS = ");
        msg += std::to_string(RMS);
        msgBoxOK.setText(msg.c_str());
        msgBoxOK.exec();
        stop();
        return RMS;
    } else
        goto novaImg;
}

void StereoCameras::stop()
{
    mDisplayCapture = false;
    cv::destroyAllWindows();
    if (mCameras.IsGrabbing())
        mCameras.StopGrabbing();
}

void StereoCameras::showDisplayCapture()
{
    mDisplayCapture = true;
}

void StereoCameras::startDisplayCapture()
{
    pthread_t tid;
    int result;
    result = pthread_create(&tid, 0, StereoCameras::callDisplayCapture, this);
    if (result == 0)
        pthread_detach(tid);
}

void *StereoCameras::displayCapture(void)
{
    while (true)
    {
        if (mCameras.IsGrabbing() && mDisplayCapture)
        {
            mDisplayCaptureMutex.lock();
            capture();
            mDisplayCaptureMutex.unlock();
            auto photoPair = getStereoPhotoPair();
            auto photo1 = photoPair->matPair.first;
            auto photo2 = photoPair->matPair.second;
            //sleep(10);

            if (!photo2.empty())
            {
                cv::resize(photo1,photo1,cv::Size(photo1.cols/3,photo1.rows/3));
                cv::resize(photo2,photo2,cv::Size(photo2.cols/3,photo2.rows/3));
                cv::imshow(mCameraNames[0], photo1);
                cv::imshow(mCameraNames[1], photo2);
            }
        }
    }
}
