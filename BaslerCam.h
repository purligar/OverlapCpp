#pragma once
#ifdef PYLON
#include <ostream>

// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>
#undef PYLON_WIN_BUILD
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif
#include <pylon/ConfigurationEventHandler.h>
#include <pylon/ImageEventHandler.h>
#include <pylon/GrabResultPtr.h>

class COverlap;

namespace Pylon
{    
    class CInstantCamera;

    //Example of an image event handler.
    class CSampleImageEventHandler : public CImageEventHandler 
    {
    private:
        COverlap* m_ot;

    public:     
        CSampleImageEventHandler(COverlap* pOT)
            : CImageEventHandler() , m_ot(pOT)
        {
            
        }
    public:
        virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult);
    };

    class CImageEventPrinter : public CImageEventHandler
    {
    private:
        COverlap* m_ot;

    public:
        CImageEventPrinter(COverlap* pOT)
            : CImageEventHandler(), m_ot(pOT)
        {}
        virtual void OnImagesSkipped(CInstantCamera& camera, size_t countOfSkippedImages);

        virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult);
    };

    class CConfigurationEventPrinter : public CConfigurationEventHandler
    {
    public:
        void OnAttach(CInstantCamera& /*camera*/)
        {
            std::cout << "OnAttach event" << std::endl;
        }

        void OnAttached(CInstantCamera& camera)
        {
            std::cout << "OnAttached event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnOpen(CInstantCamera& camera)
        {
            std::cout << "OnOpen event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnOpened(CInstantCamera& camera)
        {
            std::cout << "OnOpened event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnGrabStart(CInstantCamera& camera)
        {
            std::cout << "OnGrabStart event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnGrabStarted(CInstantCamera& camera)
        {
            std::cout << "OnGrabStarted event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnGrabStop(CInstantCamera& camera)
        {
            std::cout << "OnGrabStop event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnGrabStopped(CInstantCamera& camera)
        {
            std::cout << "OnGrabStopped event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnClose(CInstantCamera& camera)
        {
            std::cout << "OnClose event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnClosed(CInstantCamera& camera)
        {
            std::cout << "OnClosed event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnDestroy(CInstantCamera& camera)
        {
            std::cout << "OnDestroy event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnDestroyed(CInstantCamera& /*camera*/)
        {
            std::cout << "OnDestroyed event" << std::endl;
        }

        void OnDetach(CInstantCamera& camera)
        {
            std::cout << "OnDetach event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnDetached(CInstantCamera& camera)
        {
            std::cout << "OnDetached event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }

        void OnGrabError(CInstantCamera& camera, const char* errorMessage)
        {
            std::cout << "OnGrabError event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
            std::cout << "Error Message: " << errorMessage << std::endl;
        }

        void OnCameraDeviceRemoved(CInstantCamera& camera)
        {
            std::cout << "OnCameraDeviceRemoved event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        }
    };

    class BaslerCam //final : public eos::cam::detail::BaseCam
        //public Pylon::CSampleImageEventHandler             // Allows you to get informed about received images and grab errors.
        //, public Pylon::CConfigurationEventHandler     // Allows you to get informed about device removal.
    {
    public: // methods
        BaslerCam();
        virtual ~BaslerCam();

        int runBaslerPylonCamera(COverlap* pOt);

        /*bool supportsBuffering() const override;
        bool getBufferSizeRange(size_t& bufferSizeMin, size_t& bufferSizeMax) const override;
        bool getBufferSize(size_t& bufferSize) const override;
        bool supportsTriggerMode(TriggerMode::Type triggerMode) const override;*/

    private:
        //std::string getImplementationId() const override;
        //bool isAvailableImpl() const override;
        //bool openImpl() override;
        //bool allocateMemory() override;

        //bool initialize() override;
        //bool changeTriggerModeImpl(TriggerMode::Type triggerMode) override;

        //void closeImpl() override;
        //void triggerNextFrame() override;
        //bool startCamera();
        //bool stopCamera();

        //bool setExposureTimeImpl(double _exposureTime) override;
        //bool getExposureTimeImpl(double& _exposureTime) const override;
        //bool getExposureTimeRangeImpl(double& _exposureTimeMin, double& _exposureTimeMax) const override;

        //bool setDelayImpl(double _delay) override;
        //bool getDelayImpl(double& _delay) const override;

        //bool setGainImpl(unsigned int _gain) override;
        //bool getGainImpl(unsigned int& _gain) const override;
        //bool getGainRangeImpl(unsigned int& _gainMin, unsigned int& _gainMax) const override;

        //bool getFrameRateImpl(double& frameRate) const override;
        //bool setFrameRateImpl(double frameRate) override;

        //bool getBlacklevelOffsetImpl(int& offset) const override;
        //bool setBlacklevelOffsetImpl(int offset) override;
        //bool getBlacklevelOffsetRangeImpl(int& offsetMin, int& offsetMax) const override;

        //bool getGammaImpl(double& _gamma) const override;
        //bool setGammaImpl(double _gamma) const override;

        //bool getMedianFilterEnabledImpl(bool& _enabled) const override;
        //bool setMedianFilterEnabledImpl(bool _enabled) override;

        //bool setBufferSizeImpl(size_t bufferSize) override;

        //bool loadParametersImpl(Camera::ParameterSetLocation location) override;
        //bool saveParametersImpl(Camera::ParameterSetLocation location) const override;


        //bool getSupportedColorModesImpl(std::vector<Camera::ColorMode::Type>& _supportedColorModes) override;
        //bool getColorModeImpl(Camera::ColorMode::Type& _colorMode) const override;
        //bool setColorModeImpl(Camera::ColorMode::Type _colorMode) override;

        //bool getAdditionalParameterImpl(std::string parameter, std::string& value) const override;
        //bool setAdditionalParameterImpl(std::string parameter, std::string value) override;

        //// Pylon::CImageEventHandler functions
        //void OnImagesSkipped(Pylon::CInstantCamera& camera, size_t countOfSkippedImages) override;
        //void OnImageGrabbed(Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& grabResult) override;
        //void OnCameraDeviceRemoved(Pylon::CInstantCamera& camera) override;

        //bool getCameraVersionInfoImpl(CameraVersionInfo& cameraVersionInfo) const override;

        //static bool getBitsPerPixel(Basler_UniversalCameraParams::PixelFormatEnums colorMode, INT& nBitsPerPixel, INT& numberOfChannels);

        //bool getTemperatureImpl(double& _temperature) const override;

        //std::string getDeviceSerial() const;

        //void checkMissingImages(const Pylon::CGrabResultPtr& grabResult);

        ///// <summary>
        ///// Sets the transport layer packet size.
        ///// https://docs.baslerweb.com/network-related-parameters GevSCPSPacketSize
        ///// </summary>
        ///// <param name="value"> Value is set as packet size.</param>
        ///// <returns>True if packet size could be retrieved successfully</returns>
        //bool getPacketSize(int64_t& value) const;

        ///// <summary>
        ///// Gets the transport layer packet size.
        ///// https://docs.baslerweb.com/network-related-parameters GevSCPSPacketSize
        ///// </summary>
        ///// <param name="value"> Value is set to packet size.</param>
        ///// <returns>True if packet size could be parsed and set successfully</returns>
        //bool setPacketSize(uint64_t value);

        ///// <summary>
        ///// Gets the delay between ethernet packages.
        ///// https://docs.baslerweb.com/network-related-parameters#inter-packet-delay GevSCPD
        ///// </summary>
        ///// <param name="value"> Value is set as packet size.</param>
        ///// <returns>True if packet delay could be retrieved successfully</returns>
        //bool getPacketDelay(int64_t& value) const;

        ///// <summary>
        ///// Sets the delay between ethernet packages.
        ///// https://docs.baslerweb.com/network-related-parameters#inter-packet-delay GevSCPD
        ///// </summary>
        ///// <param name="value"> Value is set as packet delay.</param>
        ///// <returns>True if packet delay could be parsed and set successfully</returns>
        //bool setPacketDelay(uint64_t value);

        ///// <summary>
        ///// Gets the packet timeout after which the camera will resend a package.
        ///// https://docs.baslerweb.com/stream-grabber-parameters#packet-timeout
        ///// </summary>
        ///// <param name="value"></param>
        ///// <returns></returns>
        //bool getPacketTimeout(int64_t& value) const;

        ///// <summary>
        ///// Sets the packet timeout after which the camera will resend a package.
        ///// https://docs.baslerweb.com/stream-grabber-parameters#packet-timeout
        ///// </summary>
        ///// <param name="value"></param>
        ///// <returns></returns>
        //bool setPacketTimeout(uint64_t value);

        ///// <summary>
        ///// Gets the frame retention duration which specifies the maximum time in milliseconds to receive all packets of a frame. This parameter is reset to defaults when camera is (re)-opened.
        ///// https://docs.baslerweb.com/stream-grabber-parameters#frame-retention
        ///// </summary>
        ///// <param name="value"></param>
        ///// <returns></returns>
        //bool getFrameRetention(int64_t& value) const;

        ///// <summary>
        ///// Gets the frame retention duration which specifies the maximum time in milliseconds to receive all packets of a frame. This parameter is reset to defaults when camera is (re)-opened.
        ///// https://docs.baslerweb.com/stream-grabber-parameters#frame-retention
        ///// </summary>
        ///// <param name="value"></param>
        ///// <returns></returns>
        //bool setFrameRetention(uint64_t value);

        ///// <summary>
        ///// Sets a specific test pattern to be provided by the camera. Not all test patterns are supported by all cameras.
        ///// https://docs.baslerweb.com/test-patterns
        ///// TestPattern_Black = 0,
        ///// TestPattern_ColorDiagonalSawtooth8 = 1,
        ///// TestPattern_GreyDiagonalSawtooth8 = 2,
        ///// TestPattern_Off = 3,
        ///// TestPattern_Testimage1 = 4,
        ///// TestPattern_Testimage2 = 5,
        ///// TestPattern_Testimage3 = 6, 
        ///// TestPattern_Testimage6 = 7,
        ///// TestPattern_White = 8
        ///// </summary>
        ///// <param name="value"></param>
        ///// <returns></returns>
        //bool setTestPattern(Basler_UniversalCameraParams::TestPatternEnums value);

        //bool getCameraDetailsImpl(std::vector<std::pair<std::string, std::string>>& details) const override;
        //bool getTransportProtocolImpl(TransportProtocol::Type& transportProtocol) const override;
        //bool getConnectedHostImpl(std::string& value) const override;

        ///// <summary>
        ///// Set provided static IP configuration. We broadcast on all networks to reach the camera. 
        ///// The camera to be changed is identified via mac address. This method stops and closes 
        ///// the camera. When re-opening the camera all previous settings are lost and need to be set again.
        ///// Will return false if ip is not in range of a connected network or the ip is already in use.
        ///// </summary>
        ///// <param name="ipString"></param>
        ///// <param name="subnetMask"></param>
        ///// <param name="defaultGateway"></param>
        ///// <returns></returns>
        //bool enableStaticIpImpl(const std::string& ipString, const std::string& subnetMask, const std::string& defaultGateway) override;
        //bool enableDhcpIpImpl() override;

        //bool broadcastIpConfiguration(bool enableDhcp, const std::string& ipAddress, const std::string& subnetMask, const std::string& defaultGateway);

        ///// <summary>
        ///// Formats macAddress string to readable format. 00305345FD08 -> 00:30:53:45:FD:08
        ///// </summary>
        ///// <param name="macAddressPylon"></param>
        ///// <returns></returns>
        //std::string formatToReadableMacAddress(const Pylon::String_t& macAddressPylon);

        ///// <summary>
        ///// Converts string representation of ColorFormatEnum to Camera::ColorMode enum. 
        ///// </summary>
        ///// <param name="pixelFormatEnumString">E.g: "Mono12p" or "Mono8"</param>
        ///// <returns></returns>
        //Camera::ColorMode::Type cvtPixelFormatEnumStringToColorMode(const std::string& pixelFormatEnumString) const;

    private: // static constants
        static size_t constexpr                 MAXIMUM_BUFFER_SIZE = 32767;
        static size_t constexpr                 MINIMUM_BUFFER_SIZE = 3;

    private: // variables
        // The camera
        //Pylon::CBaslerUniversalInstantCamera    m_camera;

        ////! Whether median filtering is enabled
        //bool                                    m_medianFilterUsed;

        ////! Error message to log file
        //bool                                    m_ErrorToLog;

        //! Next Expected Frame Number Image
        uint64_t                                m_nextExpectedFrameNumberImage;

        //std::unique_ptr<ImageFormatConverter>   m_imageFormatConverter{};

    };
    
}

int runBaslerPylonCamera(COverlap* pOt);
#endif