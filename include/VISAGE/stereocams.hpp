#ifndef VISAGE_STEREOCAMS_HPP
#define VISAGE_STEREOCAMS_HPP

#include <VISAGE/baslerconfig.hpp>

#include <opencv2/core/core.hpp>

#include <array>
#include <vector>
#include <string>
#include <utility>
#include <memory>
#include <mutex>


class StereoCameras
{
    public:
        StereoCameras();

        struct StereoPhoto
        {
            std::array<std::string, 2>  cameras;
            std::pair<cv::Mat, cv::Mat> matPair;
        };

        double CalibrarStCam(float tamQuad, cv::Size TamTab);


    public:
        void                        exec();
        StereoPhoto*                getStereoPhotoPair();
        void                        capture();
        void                        stop();
        void                        startDisplayCapture();
        void                        showDisplayCapture();


    private:
        void*                       displayCapture(void);
        static void*                callDisplayCapture(void *arg){return ((StereoCameras*)arg)->displayCapture();}


    private:
        bool                        attachDevices();
        void                        registerCameraCapture(StereoPhoto* StereoPhotoPtr);
        cv::Mat CM1 = cv::Mat(3, 3, CV_64FC1);
        cv::Mat CM2 = cv::Mat(3, 3, CV_64FC1);
        cv::Mat D1, D2;
        cv::Mat R, T, E, F;
        cv::Mat R1, R2, P1, P2, Q;


    private:
        std::unique_ptr<StereoCameras::StereoPhoto> mStereoPhotoPtr;
        Pylon::PylonAutoInitTerm                    mAutoInitTerm;
        Pylon::CTlFactory&                          mTransportLayerFactory;
        Pylon::DeviceInfoList_t                     mDevices;
        Pylon::CInstantCameraArray                  mCameras;
        std::vector<std::string>                    mCameraNames;
        bool                                        mDisplayCapture;
        std::mutex                                  mDisplayCaptureMutex;
};

// Forward Declaration
namespace Pylon
{
    class CInstantCamera;
    class CGrabResultPtr;
}

class CameraCapture : public Pylon::CImageEventHandler
{
    public:
                                            CameraCapture(std::string cameraName, StereoCameras::StereoPhoto* stereoPhotoPtr);

        virtual void    					OnImageGrabbed(Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& grabResultPtr);        


    private:
        std::string							mCameraName;
        std::array<cv::Mat, 5>				mCalibrationMatrices;
        const std::array<std::string, 5>	mCalibrationMatricesFiles;
        const std::array<std::string, 5>	mCalibrationMatricesNames;
        cv::Size                            mPatternSize;
        StereoCameras::StereoPhoto*         mStereoPhotoPtr;
        float                               mThreshold;
};

#endif // VISAGE_STEREOCAMS_HPP
