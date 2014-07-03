#ifndef VISAGE_STEREOCAMS_HPP
#define VISAGE_STEREOCAMS_HPP

#include <pylon/TlFactory.h>
#include <pylon/InstantCameraArray.h>
#include <pylon/ConfigurationEventHandler.h>
#include <pylon/ImageEventHandler.h>

#include <opencv2/core/core.hpp>

#include <array>
#include <vector>
#include <string>
#include <utility>
#include <memory>


class StereoCameras
{
    public:
        StereoCameras();

        struct StereoPhoto
        {
            std::array<std::string, 2>  cameras;
            std::pair<cv::Mat, cv::Mat> matPair;
        };


    public:
        void                        exec();
        StereoPhoto*                getStereoPhotoPair();
        void                        capture();


    private:
        bool                        attachDevices();
        void                        registerCameraCapture(StereoPhoto* StereoPhotoPtr);


    private:
        std::unique_ptr<StereoCameras::StereoPhoto> mStereoPhotoPtr;
        Pylon::PylonAutoInitTerm                    mAutoInitTerm;
        Pylon::CTlFactory&                          mTransportLayerFactory;
        Pylon::DeviceInfoList_t                     mDevices;
        Pylon::CInstantCameraArray                  mCameras;
        std::vector<std::string>                    mCameraNames;
};

// Forward Declaration
namespace Pylon
{
    class CInstantCamera;
    class CGrabResultPtr;
}

class CameraConfiguration : public Pylon::CConfigurationEventHandler
{
    public:
                            CameraConfiguration(const char* configurationFile, const int interPacketDelay, int frameTransmissionDelay, std::string cameraName);
        void                OnOpened(Pylon::CInstantCamera& camera);
        void                OnGrabStarted(Pylon::CInstantCamera& camera);


    private:
        const char*         mConfigurationFile;
        const int           mInterPacketDelay;
        const int           mFrameTransmissionDelay;
        const std::string   mCameraName;
};

namespace
{
    const int Q 	= 0;
    const int MX1 	= 1;
    const int MY1 	= 2;
    const int MX2	= 3;
    const int MY2 	= 4;
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
