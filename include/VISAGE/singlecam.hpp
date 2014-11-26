#ifndef VISAGE_SINGLECAM_HPP
#define VISAGE_SINGLECAM_HPP

#include <VISAGE/baslerconfig.hpp>

#include <opencv2/core/core.hpp>

#include <array>
#include <vector>
#include <string>
#include <utility>
#include <memory>
#include <mutex>

class CameraBasler
{
public:
    CameraBasler(std::string ip);

    struct Photo
    {
        std::string camera;
        cv::Mat mat;
    };

public:
    void                        exec();
    Photo*                      getPhoto();
    void                        capture();
    void                        stop();
    void                        startDisplayCapture();
    void                        showDisplayCapture();
    bool                        isGrabbing();
    std::string                 getName();


private:
    void*                       displayCapture(void);
    static void*                callDisplayCapture(void *arg){return ((CameraBasler*)arg)->displayCapture();}


private:
    bool                        attachDevice(std::string ip);
    void                        registerCameraCapture(Photo* PhotoPtr);
    cv::Mat CM1 = cv::Mat(3, 3, CV_64FC1);
    cv::Mat CM2 = cv::Mat(3, 3, CV_64FC1);
    cv::Mat D1, D2;
    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q;



private:
    std::unique_ptr<CameraBasler::Photo>        mPhotoPtr;
    Pylon::PylonAutoInitTerm                    mAutoInitTerm;
    Pylon::CTlFactory&                          mTransportLayerFactory;
    Pylon::DeviceInfoList_t                     mDevices;
    Pylon::CInstantCamera                       mCamera;
    std::string                                 mCameraName;
    std::mutex                                  mDisplayCaptureMutex;
};

class CameraCaptureMono : public Pylon::CImageEventHandler
{
public:
    CameraCaptureMono(std::string cameraName, CameraBasler::Photo* photoPtr);

    virtual void    					OnImageGrabbed(Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& grabResultPtr);


private:
    std::string							mCameraName;
    std::array<cv::Mat, 5>				mCalibrationMatrices;
    const std::array<std::string, 5>	mCalibrationMatricesFiles;
    const std::array<std::string, 5>	mCalibrationMatricesNames;
    cv::Size                            mPatternSize;
    CameraBasler::Photo*                mPhotoPtr;
    float                               mThreshold;
};


#endif // VISAGE_SINGLECAM_HPP
