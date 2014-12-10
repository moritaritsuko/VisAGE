#include <VISAGE/singlecam.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <unistd.h>
#include <iostream>

#include <QMessageBox>


CameraBasler::CameraBasler(std::string ip)
    : mPhotoPtr(new CameraBasler::Photo)
    , mAutoInitTerm()
    , mTransportLayerFactory(Pylon::CTlFactory::GetInstance())
    , mDevices()
    , mCamera()
    , mCameraName()
    , mCameraIp(ip)
    , mDisplayCaptureMutex()
    , mMatCaptureMutex()
{
    if (!attachDevice())
        std::cout << "AVISO: Câmera não foi encontrada." << std::endl;
}

void CameraBasler::exec()
{
    mCamera.Open();
    if (mCamera.IsOpen())
    {
        auto Photo = mPhotoPtr.get();
        registerCameraCapture(Photo);
        // Cameras Synchronization: Round-Robin Strategy
        mCamera.StartGrabbing(Pylon::GrabStrategy_UpcomingImage);
        startDisplayCapture();
    }
}

void CameraBasler::execMat()
{
    mCamera.Open();
    if (mCamera.IsOpen())
    {
        auto Photo = mPhotoPtr.get();
        registerCameraCapture(Photo);
        // Cameras Synchronization: Round-Robin Strategy
        mCamera.StartGrabbing(Pylon::GrabStrategy_UpcomingImage);
    }
}

CameraBasler::Photo* CameraBasler::getPhoto()
{
    return mPhotoPtr.get();
}

bool CameraBasler::attachDevice()
{
    if (mTransportLayerFactory.EnumerateDevices(mDevices) == 0)
        return false;

    for (size_t i = 0; i < mDevices.size(); ++i)
    {
        std::string cameraName;
        // Attach device to Pylon's camera array
        auto device = mTransportLayerFactory.CreateDevice(mDevices[i]);
        auto device_ip = device->GetDeviceInfo().GetFullName();

        if (device_ip.find("169.254.8.106") != std::string::npos)
        {
            mCamera.Attach(mTransportLayerFactory.CreateDevice(mDevices[i]));
            Pylon::CInstantCamera &camera = mCamera;
            // Define CameraBasler Name for logging and OpenCV NamedWindow
            cameraName = camera.GetDeviceInfo().GetFullName();
            mCameraName = cameraName;
            // Register CameraBasler's Configuration
            camera.RegisterConfiguration
                    (
                        //new CameraConfiguration("config/default_linux.pfs", 8192, 4096 * (int) (i + 1), cameraName),
                        new CameraConfiguration("config/default_linux.pfs", 0, 0, cameraName),
                        Pylon::RegistrationMode_ReplaceAll,
                        Pylon::Cleanup_Delete
                        );
            break;
        }
    }
    return true;
}

void CameraBasler::registerCameraCapture(CameraBasler::Photo* PhotoPtr)
{
    PhotoPtr->camera = mCameraName;
    mCamera.RegisterImageEventHandler
            (
                new CameraCaptureMono(mCameraName, PhotoPtr),
                Pylon::RegistrationMode_ReplaceAll,
                Pylon::Cleanup_Delete
                );
}

void CameraBasler::capture()
{
    Pylon::CGrabResultPtr grabResultPtr;

    if (mCamera.IsGrabbing())
    {
        // Triggers Capture Event
        mCamera.RetrieveResult(5000, grabResultPtr, Pylon::TimeoutHandling_ThrowException);
    }
}

CameraCaptureMono::CameraCaptureMono(std::string cameraName, CameraBasler::Photo* PhotoPtr)
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
    , mPhotoPtr(PhotoPtr)
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

void CameraCaptureMono::OnImageGrabbed(Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& grabResultPtr)
{
    if (grabResultPtr->GrabSucceeded())
    {
        // OpenCV image CV_8U: 8-bits, 1 channel
        auto imageCamera = cv::Mat(grabResultPtr->GetHeight(), grabResultPtr->GetWidth(), CV_8UC1, grabResultPtr->GetBuffer());
        auto image = cv::Mat(grabResultPtr->GetHeight(), grabResultPtr->GetWidth(), CV_8UC1);
        auto cameraContextValue = grabResultPtr->GetCameraContext();

        cv::cvtColor(imageCamera, imageCamera, CV_BayerGB2RGB);

        mPhotoPtr->mat = imageCamera;
    }
    else
    {
        std::cout << mCameraName << "CameraCaptureMono::OnImageGrabbed() ERROR: " << grabResultPtr->GetErrorCode() << " " << grabResultPtr->GetErrorDescription() << std::endl;
    }
}

void CameraBasler::stop()
{
    cv::destroyAllWindows();
    if (mCamera.IsGrabbing())
        mCamera.Close();
    sleep(3);
}

void CameraBasler::startDisplayCapture()
{
    pthread_t tid;
    int result;
    result = pthread_create(&tid, 0, CameraBasler::callDisplayCapture, this);
    if (result == 0)
        pthread_detach(tid);
}

void *CameraBasler::displayCapture(void)
{
    if (mCamera.IsGrabbing())
    {
        mDisplayCaptureMutex.lock();
        capture();
        mDisplayCaptureMutex.unlock();
        auto photo = getPhoto()->mat;

        if (!photo.empty())
        {
            cv::resize(photo,photo,cv::Size(photo.cols/3,photo.rows/3));
            cv::imshow(mCameraName, photo);
        }
    }
}

cv::Mat CameraBasler::matCapture()
{
    if (mCamera.IsGrabbing())
    {
        mMatCaptureMutex.lock();
        capture();
        mMatCaptureMutex.unlock();
        auto photo = getPhoto()->mat;

        if (!photo.empty())
            return photo;
    }
}

bool CameraBasler::isGrabbing()
{
    return mCamera.IsGrabbing();
}

std::string CameraBasler::getName()
{
    return mCameraName;
}
