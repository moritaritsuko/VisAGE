#include <VISAGE/stereocams.hpp>

#include <pylon/InstantCamera.h>
#include <pylon/FeaturePersistence.h>
#include <pylon/GrabResultPtr.h>

#include <GenApi/INodeMap.h>
#include <GenApi/Types.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

#include <QMessageBox>

StereoCameras::StereoCameras()
    : mStereoPhotoPtr(new StereoCameras::StereoPhoto)
    , mAutoInitTerm()
    , mTransportLayerFactory(Pylon::CTlFactory::GetInstance())
    , mDevices()
    , mCameras(2u)
    , mCameraNames()
{
    if (attachDevices())
    {
        // Triggers Configuration Event (CameraConfiguration.cpp)
        mCameras.Open();
    }
    else
        std::cout << "AVISO: Câmeras Estéreo não foram encontradas." << std::endl;
}

void StereoCameras::exec()
{
    if (mCameras.IsOpen())
    {
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

    for (size_t i = 0; i < mDevices.size(); ++i)
    {
        std::string cameraName, cameraModel;
        // Attach device to Pylon's camera array
        mCameras[i].Attach(mTransportLayerFactory.CreateDevice(mDevices[i]));
        Pylon::CInstantCamera &camera = mCameras[i];
        // Define Camera Name for logging and OpenCV NamedWindow
        i % 2 == 0 ? cameraName = "Camera 1" : cameraName = "Camera 2";
        cameraModel += camera.GetDeviceInfo().GetModelName();
        mCameraNames.push_back(cameraName);
        // Register Camera's Configuration
        camera.RegisterConfiguration
                (
                    new CameraConfiguration("config/default_linux.pfs", 8192, 4096 * (int) (i + 1), cameraName),
                    Pylon::RegistrationMode_ReplaceAll,
                    Pylon::Cleanup_Delete
                    );
    }
    return true;
}

void StereoCameras::registerCameraCapture(StereoCameras::StereoPhoto* stereoPhotoPtr)
{
    for (size_t i = 0; i < mDevices.size(); ++i)
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

CameraConfiguration::CameraConfiguration(const char* configurationFile, const int interPacketDelay, int frameTransmissionDelay, std::string cameraName)
    : mConfigurationFile(configurationFile)
    , mInterPacketDelay(interPacketDelay)
    , mFrameTransmissionDelay(frameTransmissionDelay)
    , mCameraName(cameraName)
{
}

void CameraConfiguration::OnOpened(Pylon::CInstantCamera& camera)
{
    GenApi::INodeMap& nodeMap = camera.GetNodeMap();
    Pylon::CFeaturePersistence::Load(mConfigurationFile, &nodeMap, true);

    std::cout << "Attached and Opened " << mCameraName << std::endl;
    std::cout << "Loaded default configurations for " << camera.GetDeviceInfo().GetModelName() << std::endl;

    std::cout << "Area Of Interest (AOI) Settings:" << std::endl;
    std::cout << "Width: " << GenApi::CIntegerPtr(nodeMap.GetNode("Width"))->GetValue() << std::endl;
    std::cout << "Height: " << GenApi::CIntegerPtr(nodeMap.GetNode("Height"))->GetValue() << std::endl;
    std::cout << "Offset X: " << GenApi::CIntegerPtr(nodeMap.GetNode("OffsetX"))->GetValue() << std::endl;
    std::cout << "Offset Y: " << GenApi::CIntegerPtr(nodeMap.GetNode("OffsetY"))->GetValue() << std::endl;
    std::cout << std::endl;

    std::cout << "Pixel Format: " << GenApi::CEnumerationPtr(nodeMap.GetNode("PixelFormat"))->ToString() << std::endl;
    std::cout << std::endl;

    std::cout << "Packet Size: " << GenApi::CIntegerPtr(nodeMap.GetNode("GevSCPSPacketSize"))->GetValue() << std::endl;
    GenApi::CIntegerPtr interpacketDelay(nodeMap.GetNode("GevSCPD"));
    interpacketDelay->SetValue(mInterPacketDelay);
    GenApi::CIntegerPtr frameTransmissionDelay(nodeMap.GetNode("GevSCFTD"));
    frameTransmissionDelay->SetValue(mFrameTransmissionDelay);
    std::cout << "Inter-Packet Delay: " << interpacketDelay->GetValue() << std::endl;
    std::cout << "Frame Transmission Delay: " << frameTransmissionDelay->GetValue() << std::endl;
}

void CameraConfiguration::OnGrabStarted(Pylon::CInstantCamera& camera)
{
    std::cout << mCameraName << " is Capturing." << std::endl;
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

            auto leftImage = mStereoPhotoPtr->matPair.first;
            auto rightImage = mStereoPhotoPtr->matPair.second;
            auto leftCamera = mStereoPhotoPtr->cameras[0];
            auto rightCamera = mStereoPhotoPtr->cameras[1];
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
    msgBoxint.setText("Caçibração iniciada!");
    msgBoxint.exec();

novaImg:
    capture();
    auto photoPair = getStereoPhotoPair();
    auto photo1 = photoPair->matPair.first;
    auto photo2 = photoPair->matPair.second;
    photo1.copyTo(imgE);
    photo2.copyTo(imgD);

    cv::Mat imgGrayE;
    cv::Mat imgGrayD;
    cv::Mat cinzaMenor;
    cv::cvtColor(imgE,imgGrayE,CV_RGB2GRAY);
    cv::resize(imgGrayE,cinzaMenor,cv::Size(imgGrayE.cols/fs,imgGrayE.rows/fs));
    cv::Mat imgThreshM=cv::Mat(cinzaMenor.rows, cinzaMenor.cols, CV_8UC1);
    cv::adaptiveThreshold(cinzaMenor,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
    int found1 = findChessboardCorners( imgThreshM, TamTab, pointbufE,
                                        CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);
    cinzaMenor.copyTo(copiaImgE);

    cv::cvtColor(imgD,imgGrayD,CV_RGB2GRAY);
    cv::resize(imgGrayD,cinzaMenor,cv::Size(imgGrayD.cols/fs,imgGrayD.rows/fs));
    imgThreshM=cv::Mat(cinzaMenor.rows, cinzaMenor.cols, CV_8UC1);
    cv::adaptiveThreshold(cinzaMenor,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
    int found2 = findChessboardCorners( imgThreshM, TamTab, pointbufE,
                                        CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);
    cinzaMenor.copyTo(copiaImgD);

    if(found1 && found2){


        cv::drawChessboardCorners( copiaImgE, TamTab, cv::Mat(pointbufE), found1 );
        cv::drawChessboardCorners( copiaImgD, TamTab, cv::Mat(pointbufD), found2 );

        cv::imshow("testeD",copiaImgD);
        cv::imshow("testeE",copiaImgE);
        cv::waitKey();

        QMessageBox msgBox;
        msgBox.setText("Pontos encontrados!");
        msgBox.setInformativeText("Salvar pontos para Calibração?");
        msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard);
        msgBox.setDefaultButton(QMessageBox::Save);
        int ret = msgBox.exec();

        if(ret == QMessageBox::Save){
            imgThreshM=cv::Mat(imgGrayE.rows, imgGrayE.cols, CV_8UC1);

            cv::adaptiveThreshold(imgGrayE,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
            cv::imshow("imgThreshME",imgThreshM);
            int foundE = findChessboardCorners( imgThreshM, TamTab, pointbufE,
                                                CV_CALIB_CB_ADAPTIVE_THRESH  | CV_CALIB_CB_NORMALIZE_IMAGE);

            cv::adaptiveThreshold(imgGrayD,imgThreshM,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,47,15);
            cv::imshow("imgThreshMD",imgThreshM);
            int foundD = findChessboardCorners( imgThreshM, TamTab, pointbufD,
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
            }else{
                QMessageBox msgBoxOK;
                msgBoxOK.setText("Postos não encontrados nessa resolução!");
                msgBoxOK.exec();
            }
//            goto novaImg;
        }

        QMessageBox msgBoxCal;
        msgBoxCal.setText("Pontos Salvos!");
        msgBoxCal.setInformativeText("Calibrar?");
        msgBoxCal.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBoxCal.setDefaultButton(QMessageBox::No);
        int respCal = msgBox.exec();

        if(respCal == QMessageBox::Yes){
            goto calibrar;
        }


    }else{
        QMessageBox msgBoxOK;
        msgBoxOK.setText("Tabs não encontrados!");
        msgBoxOK.exec();
        goto novaImg;
    }

calibrar:

       std::vector<cv::Point3f> objectPoints;
       objectPoints.resize(0);

            for( int i = 0; i < TamTab.height; i++ )
                for( int j = 0; j < TamTab.width; j++ )
                    objectPoints.push_back(cv::Point3f(float(j*squareSize),
                                              float(i*squareSize), 0));



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



    return RMS;
}
