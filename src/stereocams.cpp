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
