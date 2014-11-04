#ifndef VISAGE_BASLERCONFIG_HPP
#define VISAGE_BASLERCONFIG_HPP

#include <pylon/PylonIncludes.h>

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

#endif // VISAGE_BASLERCONFIG_HPP
