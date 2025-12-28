#ifdef PYLON

#include "BaslerCam.h"

#include <opencv2/core.hpp> //Any OPENCV3 code
#include <opencv2/core/types_c.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "OTAveragingLib/OTAveraging.h"

using namespace std;

namespace Pylon
{
    void CSampleImageEventHandler::OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
        if (m_ot->m_bIsVerbose)
            std::cout << "CSampleImageEventHandler::OnImageGrabbed called." << std::endl;

        CImageFormatConverter formatConverter;//me
        formatConverter.OutputPixelFormat = PixelType_Mono8; // PixelType_BGR8packed;//me
        CPylonImage pylonImage;//me

        formatConverter.Convert(pylonImage, ptrGrabResult);//me
        // Create an OpenCV image out of pylon image
        auto openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t*)pylonImage.GetBuffer()).clone();//me

        if (m_ot != nullptr)
            m_ot->AddImage(openCvImage);

#ifdef PYLON_WIN_BUILD
        // Display the image
        Pylon::DisplayImage(1, ptrGrabResult);
#endif
    }

    void CImageEventPrinter::OnImagesSkipped(CInstantCamera& camera, size_t countOfSkippedImages)
    {
        std::cout << "OnImagesSkipped event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;
        std::cout << countOfSkippedImages << " images have been skipped." << std::endl;
        std::cout << std::endl;
    }


    void CImageEventPrinter::OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
        if (m_ot->m_bIsVerbose)
            std::cout << "OnImageGrabbed event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;

        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            if (m_ot->m_bIsVerbose)
            {
                std::cout << "SizeX: " << ptrGrabResult->GetWidth() << std::endl;
                std::cout << "SizeY: " << ptrGrabResult->GetHeight() << std::endl;
                const uint8_t* pImageBuffer = (uint8_t*)ptrGrabResult->GetBuffer();
                std::cout << "Gray value of first pixel: " << (uint32_t)pImageBuffer[0] << std::endl;
                std::cout << std::endl;
            }
        }
        else
        {
            std::cerr << "CImageEventPrinter::OnImageGrabbed::Error: " << std::hex << ptrGrabResult->GetErrorCode() << std::dec << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            m_ot->m_bStopAquisition = true;
        }
    }
    BaslerCam::BaslerCam() // (CameraInfo const& cameraInfo)
        //: BaseCam(cameraInfo)
        :
        //m_medianFilterUsed(false)
        //, m_ErrorToLog(true)
        m_nextExpectedFrameNumberImage(1)
        //m_imageFormatConverter(std::make_unique<ImageFormatConverter>())
    {
        // Before using any pylon methods, the pylon runtime must be initialized.        
        Pylon::PylonInitialize();
        cout << "PylonInitialize() called." << endl;
    }

    BaslerCam::~BaslerCam()
    {
        //LOG_TRACE("~BaslerCam");

        try
        {
            /*close();

            m_camera.DestroyDevice();
            m_imageFormatConverter.reset();*/
            Pylon::PylonTerminate();
            cout << "PylonTerminate() called." << endl;
        }
        catch (...)
        {
        }
    }


    int BaslerCam::runBaslerPylonCamera(OTAveraging* pOt)
    {
        int exitCode = 0;

        try
        {
            // Create an instant camera object for the camera device found first.
            CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

            // Print the model name of the camera.
            cout << "Using device: " << camera.GetDeviceInfo().GetModelName() << endl << endl;

            // retrieve the ConnectionGuardEnable node from the transport layer node map
            CBooleanParameter guard(camera.GetTLNodeMap(), "ConnectionGuardEnable");
            // set the value explicitly to true to always use the PylonGigEConnectionGuard. (Note: Only GigE cameras have a "ConnectionGuardEnable" node)
            guard.TrySetValue(true);

            // retrieve the heartbeat node from the transport layer node map
            CIntegerParameter heartbeat(camera.GetTLNodeMap(), "HeartbeatTimeout");
            // set heartbeat to 60 seconds. (Note: Only GigE cameras have a "HeartbeatTimeout" node)
            heartbeat.TrySetValue(60 * 1000);

            //https://docs.baslerweb.com/pylonapi/cpp/pylon_programmingguide
            /*Using the latter three approaches may require to remove the default configuration after the Instant Camera object has been created.
            The following example shows how to do this. The configuration must be removed before calling the Open() method.*/
            // Remove the default configuration
            camera.RegisterConfiguration((CConfigurationEventHandler*)NULL, RegistrationMode_ReplaceAll, Cleanup_None);
            cout << "Camera initialized with default user set 1 stored in camera !!" << endl;


            // Register the standard configuration event handler for enabling software triggering.
            // The software trigger configuration handler replaces the default configuration
            // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
            //camera.RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete );

            // For demonstration purposes only, registers an event handler configuration to print out information about camera use.
            // The event handler configuration is appended to the registered software trigger configuration handler by setting 
            // registration mode to RegistrationMode_Append.
            camera.RegisterConfiguration(new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete);

            // The image event printer serves as sample image processing.
            // When using the grab loop thread provided by the Instant Camera object, an image event handler processing the grab
            // results must be created and registered.
            camera.RegisterImageEventHandler(new CImageEventPrinter(pOt), RegistrationMode_Append, Cleanup_Delete);

            // For demonstration purposes only, register another image event handler.
            camera.RegisterImageEventHandler(new CSampleImageEventHandler(pOt), RegistrationMode_Append, Cleanup_Delete);

            // Open the camera device.
            camera.Open();

            // Can the camera device be queried whether it is ready to accept the next frame trigger?
            if (camera.CanWaitForFrameTriggerReady())
            {
                // Start the grabbing using the grab loop thread, by setting the grabLoopType parameter
                // to GrabLoop_ProvidedByInstantCamera. The grab results are delivered to the image event handlers.
                // The GrabStrategy_OneByOne default grab strategy is used.
                camera.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
                m_nextExpectedFrameNumberImage = 1;

                // Wait for user input to trigger the camera or exit the program.
                // The grabbing is stopped, the device is closed and destroyed automatically when the camera object goes out of scope.

                while (!pOt->m_bStopAquisition)
                {
                    WaitObject::Sleep(250);
                }

                if (0)
                {
                    bool runLoop = true;
                    while (runLoop)
                    {
                        cout << endl << "Enter \"t\" to trigger the camera or \"e\" to exit and press enter? (t/e) "; cout.flush();

                        string userInput;
                        getline(cin, userInput);

                        for (size_t i = 0; i < userInput.size(); ++i)
                        {
                            char key = userInput[i];
                            if ((key == 't' || key == 'T'))
                            {
                                // Execute the software trigger. Wait up to 1000 ms for the camera to be ready for trigger.
                                if (camera.WaitForFrameTriggerReady(1000, TimeoutHandling_ThrowException))
                                {
                                    camera.ExecuteSoftwareTrigger();
                                }
                            }
                            else if ((key == 'e') || (key == 'E'))
                            {
                                runLoop = false;
                                break;
                            }
                        }

                        // Wait some time to allow the OnImageGrabbed handler print its output,
                        // so the printed text on the console is in the expected order.
                        WaitObject::Sleep(250);
                    }
                }
            }
            else
            {
                // See the documentation of CInstantCamera::CanWaitForFrameTriggerReady() for more information.
                cout << endl << "This sample can only be used with cameras that can be queried whether they are ready to accept the next frame trigger." << endl;
            }
        }
        catch (const GenericException& e)
        {
            // Error handling.
            cerr << "An exception occurred." << endl << e.GetDescription() << endl;
            exitCode = 1;
        }

        // Comment the following two lines to disable waiting on exit.
        /*cerr << endl << "Press enter to exit." << endl;
        while (cin.get() != '\n');*/

        // Releases all pylon resources.

        return exitCode;
    }

//    bool BaslerCam::supportsTriggerMode(TriggerMode::Type const triggerMode) const
//    {
//        if (triggerMode == TriggerMode::SOFTWARE ||
//            triggerMode == TriggerMode::FREE_RUNNING)
//        {
//            return true;
//        }
//
//        return false;
//    }
//
//    std::string BaslerCam::getImplementationId() const
//    {
//        return "BaslerCam";
//    }
//
//    bool BaslerCam::isAvailableImpl() const
//    {
//        return true;
//    }
//
//    bool BaslerCam::openImpl()
//    {
//        LOG_TRACE(getCameraId() << ": openImpl");
//
//        try
//        {
//            // Get the transport layer factory.
//            Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
//
//            // Get all attached devices
//            Pylon::DeviceInfoList_t devices;
//            tlFactory.EnumerateDevices(devices);
//
//            Pylon::CInstantCameraArray cameras(devices.size());
//
//            for (size_t i = 0; i < cameras.GetSize(); ++i)
//            {
//                cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
//
//                if (strcmp(getCameraInfo().serial.c_str(), cameras[i].GetDeviceInfo().GetSerialNumber().c_str()) == 0)
//                {
//                    // Create the device and attach it to CInstantCamera.
//                    // Let CInstantCamera take care of destroying the device.
//                    Pylon::IPylonDevice* pDevice = Pylon::CTlFactory::GetInstance().CreateDevice(devices[i]);
//                    m_camera.Attach(pDevice);
//
//                    // Register this object as an image event handler, so we will be notified of new new images
//                    m_camera.RegisterImageEventHandler(this, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_None);
//
//#ifdef NDEBUG
//                    // Set timeout for detection of the device removal.
//                    // Setting this value will close the ethernet connection after the timeout when waiting in breakpoint. 
//                    // https://docs.baslerweb.com/pylonapi/cpp/pylon_advanced_topics#pylongigeconnectionguard
//                    Pylon::CIntegerParameter heartbeat(m_camera.GetTLNodeMap(), "HeartbeatTimeout");
//                    heartbeat.TrySetValue(1000, Pylon::IntegerValueCorrection_Nearest);  // set to 1000 ms timeout if writable
//#endif
//                    m_nextExpectedFrameNumberImage = 1;
//
//                    // Open camera.
//                    m_camera.Open();
//
//                    return true;
//                }
//            }
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": " << e.what());
//            return false;
//        }
//
//        return false;
//    }
//
//    bool BaslerCam::allocateMemory()
//    {
//        try
//        {
//            INT nBitsPerPixel = 0;
//            INT numberOfChannels = 0;
//
//            // get actual colorMode
//            const PixelFormatEnums colorMode = m_camera.PixelFormat.GetValue();
//            if (!getBitsPerPixel(colorMode, nBitsPerPixel, numberOfChannels))
//            {
//                LOG_ERROR(getCameraId() << ": Bits per pixel of the camera sensor are not supported.");
//                return false;
//            }
//
//            // Select pixel depth of target cv::Mat so data fits into it.
//            int nBytesPerPixel;
//            if (nBitsPerPixel > 0 && nBitsPerPixel <= 8)
//            {
//                nBytesPerPixel = 1;
//            }
//            else if (nBitsPerPixel > 8 && nBitsPerPixel <= 16)
//            {
//                nBytesPerPixel = 2;
//            }
//            else if (nBitsPerPixel > 16 && nBitsPerPixel <= 32)
//            {
//                nBytesPerPixel = 4;
//            }
//            else if (nBitsPerPixel > 32 && nBitsPerPixel <= 64)
//            {
//                nBytesPerPixel = 8;
//            }
//            else
//            {
//                LOG_ERROR(getCameraId() << ": Color mode of the camera sensor are not supported.");
//                return false;
//            }
//
//            if (!setFrameDims(static_cast<int>(m_camera.Width.GetValue()),
//                static_cast<int>(m_camera.Height.GetValue()), nBytesPerPixel, numberOfChannels))
//            {
//                LOG_ERROR(getCameraId() << ": Frame dimension of the camera sensor are not supported.");
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": allocateMemory failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::initialize()
//    {
//        LOG_TRACE(getCameraId() << ": initialize");
//
//        return true;
//    }
//
//    bool BaslerCam::changeTriggerModeImpl(TriggerMode::Type triggerMode)
//    {
//        LOG_TRACE(getCameraId() << ": changeTriggerModeImpl");
//
//        if (triggerMode == getTriggerMode())
//            return true;
//
//        stopCamera();
//        switch (triggerMode) {
//        case TriggerMode::SOFTWARE:
//            LOG_DEBUG(getCameraId() << ": setting trigger mode to software");
//            m_camera.RegisterConfiguration(new Pylon::CSoftwareTriggerConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
//            m_camera.RegisterConfiguration(this, Pylon::RegistrationMode_Append, Pylon::Cleanup_None);
//            break;
//        case TriggerMode::EXTERNAL:
//            LOG_ERROR(getCameraId() << ": Trigger mode not supported.");
//            break;
//        case TriggerMode::FREE_RUNNING:
//            LOG_DEBUG(getCameraId() << ": setting trigger mode to free running");
//            m_camera.RegisterConfiguration(new Pylon::CAcquireContinuousConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
//            m_camera.RegisterConfiguration(this, Pylon::RegistrationMode_Append, Pylon::Cleanup_None);
//            break;
//        case TriggerMode::UNDEFINED:
//            LOG_ERROR(getCameraId() << ": Trigger mode not supported.");
//        }
//        setTriggerMode(triggerMode);
//        startCamera();
//
//        return true;
//    }
//
//    void BaslerCam::closeImpl()
//    {
//        LOG_TRACE(getCameraId() << ": closeImpl");
//
//        try
//        {
//            // Camera may have been disconnected.
//            if (m_camera.IsGrabbing())
//            {
//                m_camera.StopGrabbing();
//            }
//
//            m_camera.Close();
//        }
//        catch (const Pylon::RuntimeException& e)
//        {
//            LOG_ERROR(getCameraId() << ": closeImpl failed, error was: " << e.what());
//        }
//    }
//
//    void BaslerCam::triggerNextFrame()
//    {
//        LOG_TRACE(getCameraId() << ": triggerNextFrame");
//
//        try
//        {
//            // Only wait if software trigger is currently turned on.
//            if (m_camera.TriggerSource.GetValue() == TriggerSource_Software
//                && m_camera.TriggerMode.GetValue() == TriggerMode_On)
//            {
//                // If the camera is currently processing a previous trigger command,
//                // it will silently discard trigger commands.
//                // We wait until the camera is ready to process the next trigger.
//                m_camera.WaitForFrameTriggerReady(3000, Pylon::TimeoutHandling_ThrowException);
//            }
//            // Send trigger
//            m_camera.ExecuteSoftwareTrigger();
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": triggerNextFrame failed, error was: " << e.what());
//        }
//    }
//
//    void BaslerCam::checkMissingImages(const Pylon::CGrabResultPtr& grabResult)
//    {
//        const uint64_t frameNumber = grabResult->GetBlockID();
//        if (frameNumber == 0 && m_ErrorToLog == true)
//        {
//            LOG_ERROR(getCameraId() << "Frame number not supported!");
//            m_ErrorToLog = false;
//        }
//
//        // BlockID is invalid
//        if (frameNumber == UINT64_MAX)
//        {
//            LOG_ERROR(getCameraId() << "Invalid frame number!");
//            m_nextExpectedFrameNumberImage = 1;
//            return;
//        }
//
//        if (frameNumber == 1 && m_nextExpectedFrameNumberImage == UINT16_MAX + 1)
//        {
//            m_nextExpectedFrameNumberImage = 2;
//            return;
//        }
//
//        // Check for missing images
//        if (frameNumber != m_nextExpectedFrameNumberImage)
//        {
//            const std::string errMsg = "Lost frames: " + std::to_string(frameNumber - m_nextExpectedFrameNumberImage);
//            LOG_ERROR(getCameraId() << ":" << errMsg);
//            signalErrorOccurred(errMsg);
//            m_nextExpectedFrameNumberImage = frameNumber;
//        }
//        m_nextExpectedFrameNumberImage++;
//    }
//
//    void BaslerCam::OnImageGrabbed(Pylon::CInstantCamera& /*camera*/, const Pylon::CGrabResultPtr& grabResult)
//    {
//        try
//        {
//            // First check whether the smart pointer is valid.
//            // Then call GrabSucceeded() on the CGrabResultData to test whether the grab result contains
//            // an successfully grabbed image.
//            // In case of i.e. transmission errors the result may be invalid
//            if (grabResult.IsValid() && grabResult->GrabSucceeded())
//            {
//                if (m_imageFormatConverter->isConversionEnabled())
//                {
//                    if (!m_imageFormatConverter->convert(grabResult, getDataPtr(), getDataSize()))
//                    {
//                        LOG_ERROR("Failed to convert or copy new frame. Discarding frame.");
//                        signalErrorOccurred("Lost frames: 1");
//                        m_nextExpectedFrameNumberImage++;
//                        increaseFrameCount();
//                        return; // Discard frame
//                    }
//                }
//                else
//                {
//                    // Copy grabResult data to cv::Mat buffer m_frame
//                    const uint64_t targetBufferSize = getDataSize();
//                    const uint64_t sourceBufferSize = grabResult->GetBufferSize();
//                    if (sourceBufferSize > targetBufferSize)
//                    {
//                        LOG_ERROR("Failed to copy new frame. Image data buffer larger than target buffer.");
//                        signalErrorOccurred("Lost frames: 1");
//                        m_nextExpectedFrameNumberImage++;
//                        increaseFrameCount();
//                        return; // Discard frame
//                    }
//
//                    if (memcpy_s(getDataPtr(), targetBufferSize, grabResult->GetBuffer(), sourceBufferSize) != 0)
//                    {
//                        LOG_ERROR(getCameraId() << "Failed to copy new frame!");
//                        signalErrorOccurred("Lost frames: 1");
//                        m_nextExpectedFrameNumberImage++;
//                        increaseFrameCount();
//                        return; // Discard frame
//                    }
//                }
//
//                if (m_medianFilterUsed)
//                {
//                    medianBlur(accessCvMat(), accessCvMat(), 3);
//                }
//
//                increaseFrameCount();
//                signalNewFrame();
//
//                checkMissingImages(grabResult);
//            }
//            else
//            {
//                LOG_ERROR(getCameraId() << ": Error code: " << grabResult->GetErrorCode()
//                    << ", Error description: " << grabResult->GetErrorDescription());
//                if (grabResult->GetErrorDescription().find("image loss") != std::string::npos)
//                {
//                    signalErrorOccurred("Lost frames: 1");
//                }
//                else
//                {
//                    signalErrorOccurred(grabResult->GetErrorDescription().c_str());
//                }
//            }
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": OnImageGrabbed failed, error was: " << e.what());
//        }
//    }
//
//    void BaslerCam::OnImagesSkipped(Pylon::CInstantCamera& /*camera*/, size_t countOfSkippedImages)
//    {
//        const std::string errMsg = "Lost frames: " + std::to_string(countOfSkippedImages);
//        LOG_ERROR(getCameraId() << ":" << errMsg);
//        signalErrorOccurred(errMsg);
//    }
//
//    void BaslerCam::OnCameraDeviceRemoved(Pylon::CInstantCamera& /*camera*/)
//    {
//        LOG_ERROR(getCameraId() << ": Disconnected.");
//        setConnectionState(ConnectionState::DISCONNECTED);
//
//        // Destroy the Pylon Device representing the detached camera device.
//        // It can't be used anymore.
//        m_camera.Close();
//    }
//
//    bool BaslerCam::startCamera()
//    {
//        LOG_TRACE(getCameraId() << ": Start Camera");
//
//        try
//        {
//            // Camera may have been disconnected.
//            if (!m_camera.IsOpen() || m_camera.IsGrabbing())
//            {
//                return false;
//            }
//
//            // Start grabbing until StopGrabbing() is called.
//            m_camera.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
//            m_nextExpectedFrameNumberImage = 1;
//        }
//        catch (const Pylon::RuntimeException& e)
//        {
//            LOG_ERROR(getCameraId() << ": startCamera failed, error was: " << e.what());
//            return false;
//        }
//
//        return true;
//    }
//
//    bool BaslerCam::stopCamera()
//    {
//        LOG_TRACE(getCameraId() << ": Stop Camera");
//
//        try
//        {
//            // Camera may have been disconnected.
//            if (m_camera.IsGrabbing())
//            {
//                m_camera.StopGrabbing();
//            }
//        }
//        catch (const Pylon::RuntimeException& e)
//        {
//            LOG_ERROR(getCameraId() << ": stopCamera failed, error was: " << e.what());
//            return false;
//        }
//
//        return true;
//    }
//
//    bool BaslerCam::setExposureTimeImpl(double exposureTime)
//    {
//        LOG_TRACE(getCameraId() << ": setExposureTimeImpl");
//
//        try
//        {
//            if (m_camera.ExposureTime.IsValid())
//            {
//                m_camera.ExposureTime.SetValue(exposureTime * 1000);
//            }
//            else if (m_camera.ExposureTimeRaw.IsValid())
//            {
//                m_camera.ExposureTimeRaw.SetValue(boost::math::iround(exposureTime * 1000));
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setExposureTimeImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getExposureTimeImpl(double& exposureTime) const
//    {
//        LOG_TRACE(getCameraId() << ": getExposureTimeImpl");
//
//        try
//        {
//            if (m_camera.ExposureTime.IsValid())
//            {
//                exposureTime = m_camera.ExposureTime.GetValue() / 1000;
//            }
//            else if (m_camera.ExposureTimeRaw.IsValid())
//            {
//                exposureTime = static_cast<double>(m_camera.ExposureTimeRaw.GetValue()) / 1000;
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getExposureTimeImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getExposureTimeRangeImpl(double& _exposureTimeMin, double& _exposureTimeMax) const
//    {
//        LOG_TRACE(getCameraId() << ": getExposureTimeRangeImpl");
//
//        try
//        {
//            if (m_camera.ExposureTime.IsValid())
//            {
//                _exposureTimeMin = m_camera.ExposureTime.GetMin() / 1000;
//                _exposureTimeMax = m_camera.ExposureTime.GetMax() / 1000;
//            }
//            else if (m_camera.ExposureTimeRaw.IsValid())
//            {
//                _exposureTimeMin = static_cast<double>(m_camera.ExposureTimeRaw.GetMin()) / 1000;
//                _exposureTimeMax = static_cast<double>(m_camera.ExposureTimeRaw.GetMax()) / 1000;
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getExposureTimeRangeImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setDelayImpl(double _delay)
//    {
//        LOG_TRACE(getCameraId() << ": setDelayImpl");
//
//        try
//        {
//            m_camera.TriggerDelay.SetValue(_delay);
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setDelayImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getDelayImpl(double& _delay) const
//    {
//        LOG_TRACE(getCameraId() << ": getDelayImpl");
//
//        try
//        {
//            _delay = m_camera.TriggerDelay.GetValue();
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getDelayImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setGainImpl(unsigned int _gain)
//    {
//        LOG_TRACE(getCameraId() << ": setGainImpl");
//
//        try
//        {
//            if (m_camera.Gain.IsValid())
//            {
//                m_camera.Gain.SetValue(_gain);
//            }
//            else if (m_camera.GainRaw.IsValid())
//            {
//                m_camera.GainRaw.SetValue(_gain);
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setGainImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getGainImpl(unsigned int& _gain) const
//    {
//        LOG_TRACE(getCameraId() << ": getGainImpl");
//
//        try
//        {
//            if (m_camera.Gain.IsValid())
//            {
//                _gain = boost::math::iround(m_camera.Gain.GetValue());
//            }
//            else if (m_camera.GainRaw.IsValid())
//            {
//                _gain = static_cast<unsigned int>(m_camera.GainRaw.GetValue());
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getGainImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getGainRangeImpl(unsigned int& _gainMin, unsigned int& _gainMax) const
//    {
//        LOG_TRACE(getCameraId() << ": getGainRangeImpl");
//
//        try
//        {
//            if (m_camera.AutoGainLowerLimit.IsValid())
//            {
//                _gainMin = boost::math::iround(m_camera.AutoGainLowerLimit.GetValue());
//            }
//            else if (m_camera.AutoGainRawLowerLimit.IsValid())
//            {
//                _gainMin = static_cast<unsigned int>(m_camera.AutoGainRawLowerLimit.GetValue());
//            }
//            else
//            {
//                return false;
//            }
//
//            if (m_camera.AutoGainUpperLimit.IsValid())
//            {
//                _gainMax = boost::math::iround(m_camera.AutoGainUpperLimit.GetValue());
//            }
//            else if (m_camera.AutoGainRawUpperLimit.IsValid())
//            {
//                _gainMax = static_cast<unsigned int>(m_camera.AutoGainRawUpperLimit.GetValue());
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getGainRangeImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getMedianFilterEnabledImpl(bool& _enabled) const
//    {
//        _enabled = m_medianFilterUsed;
//
//        return true;
//    }
//
//    bool BaslerCam::setMedianFilterEnabledImpl(bool _enabled)
//    {
//        m_medianFilterUsed = _enabled;
//
//        return true;
//    }
//
//    std::string BaslerCam::getDeviceSerial() const
//    {
//        std::string ret;
//
//        try
//        {
//            ret = m_camera.DeviceID();
//        }
//        catch (const Pylon::GenericException&)
//        {
//            try
//            {
//                ret = m_camera.DeviceSerialNumber();
//            }
//            catch (const Pylon::GenericException&)
//            {
//            }
//        }
//
//        return ret;
//    }
//
//    bool BaslerCam::getCameraVersionInfoImpl(CameraVersionInfo& cameraVersionInfo) const
//    {
//        try
//        {
//            cameraVersionInfo.firmware = m_camera.DeviceFirmwareVersion();
//            cameraVersionInfo.serial = getDeviceSerial();
//            cameraVersionInfo.hardware = m_camera.DeviceVersion();
//            cameraVersionInfo.api = PYLON_VERSIONSTRING_MAJOR R"(.)" PYLON_VERSIONSTRING_MINOR R"(.)" PYLON_VERSIONSTRING_SUBMINOR;
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getCameraVersionInfoImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getBitsPerPixel(PixelFormatEnums colorMode, INT& nBitsPerPixel, INT& numberOfChannels)
//    {
//        switch (colorMode)
//        {
//        case PixelFormat_Mono8:
//            nBitsPerPixel = 8;
//            numberOfChannels = 1;
//            break;
//        case PixelFormat_Mono10:
//        case PixelFormat_Mono12:
//        case PixelFormat_Mono16:
//            nBitsPerPixel = 16;
//            numberOfChannels = 1;
//            break;
//        case PixelFormat_Mono10p:
//            nBitsPerPixel = 10;
//            numberOfChannels = 1;
//            break;
//        case PixelFormat_Mono12p:
//            nBitsPerPixel = 12;
//            numberOfChannels = 1;
//            break;
//        case PixelFormat_BayerRG8:
//        case PixelFormat_BayerRG12:
//        case PixelFormat_BayerRG12Packed:
//        case PixelFormat_BGR10Packed:
//        case PixelFormat_BGR12Packed:
//        case PixelFormat_BGR8:
//        case PixelFormat_BGR8Packed:
//        case PixelFormat_BGRA8Packed:
//        case PixelFormat_BayerBG10:
//        case PixelFormat_BayerBG10p:
//        case PixelFormat_BayerBG12:
//        case PixelFormat_BayerBG12Packed:
//        case PixelFormat_BayerBG12p:
//        case PixelFormat_BayerBG16:
//        case PixelFormat_BayerBG8:
//        case PixelFormat_BayerGB10:
//        case PixelFormat_BayerGB10p:
//        case PixelFormat_BayerGB12:
//        case PixelFormat_BayerGB12Packed:
//        case PixelFormat_BayerGB12p:
//        case PixelFormat_BayerGB16:
//        case PixelFormat_BayerGB8:
//        case PixelFormat_BayerGR10:
//        case PixelFormat_BayerGR10p:
//        case PixelFormat_BayerGR12:
//        case PixelFormat_BayerGR12Packed:
//        case PixelFormat_BayerGR12p:
//        case PixelFormat_BayerGR16:
//        case PixelFormat_BayerGR8:
//        case PixelFormat_BayerRG10:
//        case PixelFormat_BayerRG10p:
//        case PixelFormat_BayerRG12p:
//        case PixelFormat_BayerRG16:
//        case PixelFormat_Confidence16:
//        case PixelFormat_Confidence8:
//        case PixelFormat_Coord3D_ABC32f:
//        case PixelFormat_Coord3D_C16:
//        case PixelFormat_Mono10Packed:
//        case PixelFormat_Mono12Packed:
//        case PixelFormat_Mono8Signed:
//        case PixelFormat_RGB10Packed:
//        case PixelFormat_RGB10Planar:
//        case PixelFormat_RGB10V1Packed:
//        case PixelFormat_RGB10V2Packed:
//        case PixelFormat_RGB12Packed:
//        case PixelFormat_RGB12Planar:
//        case PixelFormat_RGB12V1Packed:
//        case PixelFormat_RGB16Packed:
//        case PixelFormat_RGB16Planar:
//        case PixelFormat_RGB8:
//        case PixelFormat_RGB8Packed:
//        case PixelFormat_RGB8Planar:
//        case PixelFormat_RGBA8Packed:
//        case PixelFormat_YCbCr420_8_YY_CbCr_Semiplanar:
//        case PixelFormat_YCbCr422_8:
//        case PixelFormat_YUV411Packed:
//        case PixelFormat_YUV422Packed:
//        case PixelFormat_YUV422_8:
//        case PixelFormat_YUV422_8_UYVY:
//        case PixelFormat_YUV422_YUYV_Packed:
//        case PixelFormat_YUV444Packed:
//            return false;
//        }
//
//        return true;
//    }
//
//    bool BaslerCam::getFrameRateImpl(double& frameRate) const
//    {
//        LOG_TRACE(getCameraId() << ": getFrameRateImpl");
//
//        try
//        {
//            m_camera.AcquisitionFrameRateEnable.SetValue(true);
//
//            if (m_camera.BslResultingTransferFrameRate.IsValid())
//            {
//                // Camera family Basler ace 2
//                frameRate = m_camera.BslResultingTransferFrameRate.GetValue();
//            }
//            else if (m_camera.AcquisitionFrameRateAbs.IsValid())
//            {
//                // Camera family Basler ace
//                frameRate = m_camera.AcquisitionFrameRateAbs.GetValue();
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getFrameRateImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setFrameRateImpl(double frameRate)
//    {
//        LOG_TRACE(getCameraId() << ": setFrameRateImpl");
//
//        try
//        {
//            m_camera.AcquisitionFrameRateEnable.SetValue(true);
//
//            if (m_camera.AcquisitionFrameRate.IsValid())
//            {
//                m_camera.AcquisitionFrameRate.SetValue(frameRate);
//            }
//            else if (m_camera.AcquisitionFrameRateAbs.IsValid())
//            {
//                m_camera.AcquisitionFrameRateAbs.SetValue(frameRate);
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setFrameRateImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getBlacklevelOffsetImpl(int& offset) const
//    {
//        LOG_TRACE(getCameraId() << ": getBlacklevelOffsetImpl");
//
//        try
//        {
//            if (m_camera.BlackLevel.IsValid())
//            {
//                offset = boost::math::iround(m_camera.BlackLevel.GetValue());
//            }
//            else if (m_camera.BlackLevelRaw.IsValid())
//            {
//                offset = static_cast<int>(m_camera.BlackLevelRaw.GetValue());
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getBlacklevelOffsetImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setBlacklevelOffsetImpl(int offset)
//    {
//        LOG_TRACE(getCameraId() << ": setBlacklevelOffsetImpl");
//
//        try
//        {
//            if (m_camera.BlackLevel.IsValid())
//            {
//                m_camera.BlackLevel.SetValue(offset);
//            }
//            else if (m_camera.BlackLevelRaw.IsValid())
//            {
//                m_camera.BlackLevelRaw.SetValue(offset);
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setBlacklevelOffsetImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getBlacklevelOffsetRangeImpl(int& offsetMin, int& offsetMax) const
//    {
//        LOG_TRACE(getCameraId() << ": getBlacklevelOffsetImpl");
//
//        try
//        {
//            if (m_camera.BlackLevel.IsValid())
//            {
//                offsetMin = boost::math::iround(m_camera.BlackLevel.GetMin());
//                offsetMax = boost::math::iround(m_camera.BlackLevel.GetMax());
//            }
//            else if (m_camera.BlackLevelRaw.IsValid())
//            {
//                offsetMin = static_cast<int>(m_camera.BlackLevelRaw.GetMin());
//                offsetMax = static_cast<int>(m_camera.BlackLevelRaw.GetMax());
//            }
//            else
//            {
//                return false;
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getBlacklevelOffsetRangeImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getGammaImpl(double& _gamma) const
//    {
//        LOG_TRACE(getCameraId() << ": getGammaImpl");
//
//        try
//        {
//            if (m_camera.GammaSelector.IsValid())
//            {
//                m_camera.GammaSelector.SetValue(GammaSelector_User);
//            }
//            _gamma = m_camera.Gamma.GetValue();
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getGammaImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setGammaImpl(double _gamma) const
//    {
//        LOG_TRACE(getCameraId() << ": setGammaImpl");
//
//        try
//        {
//            if (m_camera.GammaSelector.IsValid())
//            {
//                m_camera.GammaSelector.SetValue(GammaSelector_User);
//            }
//            m_camera.Gamma.SetValue(_gamma);
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setGammaImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::supportsBuffering() const
//    {
//        return true;
//    }
//
//    bool BaslerCam::getBufferSizeRange(size_t& bufferSizeMin, size_t& bufferSizeMax) const
//    {
//        bufferSizeMin = MINIMUM_BUFFER_SIZE;
//        bufferSizeMax = MAXIMUM_BUFFER_SIZE;
//        return true;
//    }
//
//    bool BaslerCam::getBufferSize(size_t& bufferSize) const
//    {
//        LOG_TRACE(getCameraId() << ": getBufferSize");
//
//        try
//        {
//            bufferSize = static_cast<size_t>(m_camera.MaxNumBuffer.GetValueOrDefault(16));
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getBufferSize failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setBufferSizeImpl(size_t bufferSize)
//    {
//        LOG_TRACE(getCameraId() << ": setBufferSizeImpl");
//
//        try
//        {
//            m_camera.MaxNumBuffer.SetValue(static_cast<int64_t>(bufferSize));
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setBufferSizeImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getTemperatureImpl(double& _temperature) const
//    {
//        LOG_TRACE(getCameraId() << ": getTemperatureImpl");
//
//        try
//        {
//            if (m_camera.DeviceTemperature.IsValid())
//            {
//                m_camera.DeviceTemperatureSelector.SetValue(DeviceTemperatureSelector_Coreboard);
//                _temperature = m_camera.DeviceTemperature.GetValue();
//            }
//            else
//            {
//                m_camera.TemperatureSelector.SetValue(TemperatureSelector_Coreboard);
//                _temperature = m_camera.TemperatureAbs.GetValue();
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getTemperatureImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getSupportedColorModesImpl(std::vector<Camera::ColorMode::Type>& _supportedColorModes)
//    {
//        try
//        {
//            GenApi_3_1_Basler_pylon::StringList_t list;
//            m_camera.PixelFormat.GetAllValues(list);
//
//            std::vector<Camera::ColorMode::Type> supportedColors;
//            for (size_t i = 0; i < list.size(); ++i)
//            {
//                const Camera::ColorMode::Type mode = cvtPixelFormatEnumStringToColorMode(list.at(i).c_str());
//
//                // Only add ColorModes that are supported by camera lib
//                if (mode != Camera::ColorMode::UNKNOWN)
//                {
//                    supportedColors.push_back(mode);
//                }
//            }
//
//            _supportedColorModes = supportedColors;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": supportedColorModesImpl failed, error was: " << e.what());
//            return false;
//        }
//        catch (const std::exception& e)
//        {
//            LOG_ERROR("Exception occured in supportedColorModesImpl: " << e.what());
//            return false;
//        }
//
//        return true;
//    }
//
//    bool BaslerCam::getColorModeImpl(Camera::ColorMode::Type& _colorMode) const
//    {
//        LOG_TRACE(getCameraId() << ": getColorModeImpl");
//
//        try
//        {
//            PixelFormatEnums mode = m_camera.PixelFormat.GetValue();
//            switch (mode)
//            {
//            case PixelFormat_Mono8:
//                _colorMode = Camera::ColorMode::MONO8;
//                break;
//            case PixelFormat_Mono12:
//                _colorMode = Camera::ColorMode::MONO12;
//                break;
//            case PixelFormat_Mono12p:
//                _colorMode = Camera::ColorMode::MONO12p;
//                break;
//            case PixelFormat_Mono16:
//                _colorMode = Camera::ColorMode::MONO16;
//                break;
//            default:
//                return false;
//            }
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getColorModeImpl failed, error was: " << e.what());
//            return false;
//        }
//
//        return true;
//    }
//
//    bool BaslerCam::setColorModeImpl(Camera::ColorMode::Type _colorMode)
//    {
//        LOG_TRACE(getCameraId() << ": setColorModeImpl");
//
//        try
//        {
//            const bool wasGrabbing = m_camera.IsGrabbing();
//            if (wasGrabbing)
//            {
//                m_camera.StopGrabbing();
//            }
//
//            // Enabled conversions are only valid for SensorBitDepthMode=Auto 
//            // SensorBitDepthMode is not supported by camera family Basler ace U. Configuration node may not exist. It cannot be misconfigured if it does not exist.
//            // If SensorBitDepthMode=Manual it is possible to use Pixelformat_Mono12 with Pixeltype_Mono10 (running ADC 10 Bit resolution but outputting 12bit images)
//            if (m_camera.BslSensorBitDepthMode.IsValid() && m_camera.BslSensorBitDepthMode.IsReadable() &&
//                m_camera.BslSensorBitDepthMode.GetValue() == Basler_UniversalCameraParams::BslSensorBitDepthModeEnums::BslSensorBitDepthMode_Manual)
//            {
//                LOG_ERROR("SensorBitDepthMode is not set to Auto. Possible PixelFormat and PixelType mismatch. Rejecting color mode " << Camera::ColorMode::toString(_colorMode));
//                return false;
//            }
//
//            // Set Pixel format
//            PixelFormatEnums mode;
//            EPixelType pixelTypeIn;
//            EPixelType pixelTypeOut;
//            switch (_colorMode)
//            {
//            case Camera::ColorMode::MONO8:
//                mode = PixelFormat_Mono8;
//                pixelTypeIn = PixelType_Mono8;
//                pixelTypeOut = PixelType_Mono8;
//                break;
//            case Camera::ColorMode::MONO12:
//                mode = PixelFormat_Mono12;
//                pixelTypeIn = PixelType_Mono12;
//                pixelTypeOut = PixelType_Mono16;
//                break;
//            case Camera::ColorMode::MONO12p:
//                mode = PixelFormat_Mono12p;
//                pixelTypeIn = PixelType_Mono12p;
//                pixelTypeOut = PixelType_Mono16;
//                break;
//            case Camera::ColorMode::MONO16:
//                mode = PixelFormat_Mono16;
//                pixelTypeIn = PixelType_Mono16;
//                pixelTypeOut = PixelType_Mono16;
//                break;
//            default:
//                LOG_ERROR("ColorMode: " << Camera::ColorMode::toString(_colorMode) << "is not implemented.");
//                return false;
//            }
//
//            m_camera.PixelFormat.SetValue(mode);
//            if (!allocateMemory())
//            {
//                LOG_ERROR(getCameraId() << ": allocateMemory failed.");
//                return false;
//            }
//
//            m_imageFormatConverter->setupConversion(pixelTypeIn, pixelTypeOut);
//
//            if (wasGrabbing)
//            {
//                startCamera();
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setColorModeImpl failed, error was: " << e.what());
//            return false;
//        }
//        catch (const std::exception& e)
//        {
//            LOG_ERROR("Exception occured in setColorModeImpl: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getAdditionalParameterImpl(std::string parameter, std::string& value) const
//    {
//        try
//        {
//            bool success;
//            if (parameter == "GevSCPSPacketSize")
//            {
//                int64_t valueInt;
//                success = getPacketSize(valueInt);
//                if (success)
//                {
//                    value = std::to_string(valueInt);
//                }
//                return success;
//            }
//            else if (parameter == "GevSCPD")
//            {
//                int64_t valueInt;
//                success = getPacketDelay(valueInt);
//                if (success)
//                {
//                    value = std::to_string(valueInt);
//                }
//                return success;
//            }
//            else if (parameter == "PacketTimeout")
//            {
//                int64_t valueInt;
//                success = getPacketTimeout(valueInt);
//                if (success)
//                {
//                    value = std::to_string(valueInt);
//                }
//                return success;
//            }
//            else if (parameter == "FrameRetention")
//            {
//                int64_t valueInt;
//                success = getFrameRetention(valueInt);
//                if (success)
//                {
//                    value = std::to_string(valueInt);
//                }
//                return success;
//            }
//            else
//            {
//                LOG_ERROR("Get unknown parameter: " + parameter);
//                return false;
//            }
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR("GetAdditionalParameter " + parameter + ": " + e.what());
//        }
//        catch (const std::exception& e)
//        {
//            LOG_ERROR("GetAdditionalParameter " + parameter + ": " + e.what());
//        }
//        catch (...)
//        {
//            LOG_ERROR("GetAdditionalParameter " + parameter);
//        }
//        return false;
//    }
//
//    bool BaslerCam::setAdditionalParameterImpl(std::string parameter, std::string value)
//    {
//        try
//        {
//            if (parameter == "GevSCPSPacketSize")
//            {
//                return setPacketSize(std::stoll(value));
//            }
//            if (parameter == "GevSCPD")
//            {
//                return setPacketDelay(std::stoll(value));
//            }
//            if (parameter == "PacketTimeout")
//            {
//                return setPacketTimeout(std::stoll(value));
//            }
//            if (parameter == "FrameRetention")
//            {
//                return setFrameRetention(std::stoll(value));
//            }
//            if (parameter == "TestPattern")
//            {
//                return setTestPattern(static_cast<TestPatternEnums>(std::stoi(value)));
//            }
//            else
//            {
//                LOG_ERROR("Set unknown parameter: " + parameter);
//                return false;
//            }
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR("SetAdditionalParameter " + parameter + ": " + e.what());
//        }
//        catch (const std::exception& e)
//        {
//            LOG_ERROR("SetAdditionalParameter " + parameter + ": " + e.what());
//        }
//        catch (...)
//        {
//            LOG_ERROR("SetAdditionalParameter " + parameter);
//        }
//        return false;
//    }
//
//    bool BaslerCam::loadParametersImpl(Camera::ParameterSetLocation location)
//    {
//        try
//        {
//            const bool wasGrabbing = m_camera.IsGrabbing();
//            if (wasGrabbing)
//            {
//                stopCamera();
//            }
//
//            // Load parameters from desired location
//            switch (location.type)
//            {
//            case Camera::ParameterSetLocation::Type::DEFAULT:
//                m_camera.UserSetSelector.SetValue(UserSetSelector_Default);
//                m_camera.UserSetLoad.Execute();
//                break;
//            case Camera::ParameterSetLocation::Type::FILE:
//                LOG_ERROR(getCameraId() << ": ParameterSetLocation mode not supported.");
//                return false;
//            case Camera::ParameterSetLocation::Type::USER1:
//                m_camera.UserSetSelector.SetValue(UserSetSelector_UserSet1);
//                m_camera.UserSetLoad.Execute();
//                break;
//            }
//
//            // Re-allocate memory as frame size may has changed
//            if (!allocateMemory())
//            {
//                LOG_ERROR(getCameraId() << ": allocateMemory failed.");
//            }
//
//            if (wasGrabbing)
//            {
//                startCamera();
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": loadParametersImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::saveParametersImpl(Camera::ParameterSetLocation location) const
//    {
//        try
//        {
//            const bool wasGrabbing = m_camera.IsGrabbing();
//            if (wasGrabbing)
//            {
//                const_cast<BaslerCam*>(this)->stopCamera();
//            }
//
//            switch (location.type)
//            {
//            case Camera::ParameterSetLocation::Type::DEFAULT:
//                m_camera.UserSetSelector.SetValue(UserSetSelector_Default);
//                m_camera.UserSetSave.Execute();
//                break;
//            case Camera::ParameterSetLocation::Type::USER1:
//                m_camera.UserSetSelector.SetValue(UserSetSelector_UserSet1);
//                m_camera.UserSetSave.Execute();
//                break;
//            case Camera::ParameterSetLocation::Type::FILE:
//                LOG_ERROR(getCameraId() << ": ParameterSetLocation mode not supported.");
//                return false;
//            }
//
//            if (wasGrabbing)
//            {
//                const_cast<BaslerCam*>(this)->startCamera();
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": saveParametersImpl failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getPacketSize(int64_t& value) const
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) && tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": getPacketSize is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            m_camera.GevStreamChannelSelector.TrySetValue(GevStreamChannelSelector_StreamChannel0);
//            value = m_camera.GevSCPSPacketSize.GetValue();
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getPacketSize failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setPacketSize(uint64_t value)
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) && tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": setPacketSize is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            const bool wasGrabbing = m_camera.IsGrabbing();
//            if (wasGrabbing)
//            {
//                stopCamera();
//            }
//
//            m_camera.GevStreamChannelSelector.TrySetValue(GevStreamChannelSelector_StreamChannel0);
//
//            // Round to nearest valid value
//            m_camera.GevSCPSPacketSize.SetValue(value, Pylon::IntegerValueCorrection_Nearest);
//
//            if (wasGrabbing)
//            {
//                startCamera();
//            }
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setPacketSize failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getPacketDelay(int64_t& value) const
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) && tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": getPacketDelay is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            m_camera.GevStreamChannelSelector.TrySetValue(GevStreamChannelSelector_StreamChannel0);
//            value = m_camera.GevSCPD.GetValue();
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getPacketDelay failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setPacketDelay(uint64_t value)
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) && tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": setPacketDelay is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            const bool wasGrabbing = m_camera.IsGrabbing();
//            if (wasGrabbing)
//            {
//                stopCamera();
//            }
//
//            m_camera.GevStreamChannelSelector.TrySetValue(GevStreamChannelSelector_StreamChannel0);
//
//            // Round to nearest valid value
//            m_camera.GevSCPD.SetValue(value, Pylon::IntegerValueCorrection_Nearest);
//
//            if (wasGrabbing)
//            {
//                startCamera();
//            }
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setPacketDelay failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getPacketTimeout(int64_t& value) const
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) && tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": getPacketTimeout is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            Pylon::CBaslerUniversalInstantCamera* unconstCam = const_cast<Pylon::CBaslerUniversalInstantCamera*>(&m_camera);
//            value = unconstCam->GetStreamGrabberParams().PacketTimeout.GetValue();
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getPacketTimeout failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setPacketTimeout(uint64_t value)
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) && tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": setPacketTimeout is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            const bool wasGrabbing = m_camera.IsGrabbing();
//            if (wasGrabbing)
//            {
//                stopCamera();
//            }
//
//            // Round to nearest valid value
//            m_camera.GetStreamGrabberParams().PacketTimeout.SetValue(value, Pylon::IntegerValueCorrection_Nearest);
//
//            if (wasGrabbing)
//            {
//                startCamera();
//            }
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setPacketTimeout failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getFrameRetention(int64_t& value) const
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) && tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": getFrameRetention is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            Pylon::CBaslerUniversalInstantCamera* unconstCam = const_cast<Pylon::CBaslerUniversalInstantCamera*>(&m_camera);
//            value = unconstCam->GetStreamGrabberParams().FrameRetention.GetValue();
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getFrameRetention failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setFrameRetention(uint64_t value)
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) && tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": setFrameRetention is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            const bool wasGrabbing = m_camera.IsGrabbing();
//            if (wasGrabbing)
//            {
//                stopCamera();
//            }
//
//            // Round to nearest valid value
//            m_camera.GetStreamGrabberParams().FrameRetention.SetValue(value, Pylon::IntegerValueCorrection_Nearest);
//
//            if (wasGrabbing)
//            {
//                startCamera();
//            }
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setFrameRetention failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::setTestPattern(Basler_UniversalCameraParams::TestPatternEnums value)
//    {
//        try
//        {
//            const bool wasGrabbing = m_camera.IsGrabbing();
//            if (wasGrabbing)
//            {
//                stopCamera();
//            }
//
//            m_camera.TestPattern.SetValue(value);
//
//            if (wasGrabbing)
//            {
//                startCamera();
//            }
//
//            // If test pattern is not off
//            if (value != TestPattern_Off)
//            {
//                LOG_WARN("TestPattern set to: " << std::to_string(value));
//            }
//            else
//            {
//                LOG_INFO("TestPattern off.");
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": setTestPattern failed, error was: " << e.what());
//            return false;
//        }
//    }
//
//    bool BaslerCam::getCameraDetailsImpl(std::vector<std::pair<std::string, std::string>>& details) const
//    {
//        try
//        {
//            TransportProtocol::Type tp;
//            if (getTransportProtocolImpl(tp))
//            {
//                details.emplace_back("Transport Layer Type", TransportProtocol::to_string(tp));
//            }
//            else
//            {
//                return false;
//            }
//
//            // Check device is GigE
//            if (tp == TransportProtocol::GIGE)
//            {
//                // Camera ip address
//                const std::string camIp = m_camera.GetDeviceInfo().GetIpAddress();
//                details.emplace_back("Ip address", camIp);
//
//                // Camera subnetmask
//                const std::string subnetMask = m_camera.GetDeviceInfo().GetSubnetMask();
//                details.emplace_back("Subnetmask", subnetMask);
//
//                // Camera gateway
//                const std::string defaultGateway = m_camera.GetDeviceInfo().GetDefaultGateway();
//                details.emplace_back("Gateway", defaultGateway);
//
//                // Ethernet link speed 10/100/1000 Mbps
//                if (m_camera.GevLinkSpeed.IsValid() && m_camera.GevLinkSpeed.IsReadable())
//                {
//                    // Select cameras first network interface 
//                    m_camera.GevInterfaceSelector.TrySetValue(GevInterfaceSelector_NetworkInterface0);
//
//                    const int ethLinkSpeed = static_cast<int>(m_camera.GevLinkSpeed.GetValue(false, true));
//                    details.emplace_back("Linkspeed", std::to_string(ethLinkSpeed) + " Mbps");
//                }
//            }
//
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(e.what());
//        }
//        catch (const std::exception& e)
//        {
//            LOG_ERROR(e.what());
//        }
//        catch (...)
//        {
//            LOG_ERROR("Unknown error!");
//        }
//
//        return false;
//    }
//
//    bool BaslerCam::getTransportProtocolImpl(TransportProtocol::Type& transportProtocol) const
//    {
//        try
//        {
//            // https://docs.baslerweb.com/pylonapi/cpp/namespace_pylon_1_1_t_l_type
//            std::string camType = m_camera.GetDeviceInfo().GetTLType();
//
//            if (camType.empty() || camType == "N/A")
//            {
//                transportProtocol = TransportProtocol::UNKNOWN;
//                return true;
//            }
//            if (camType == Pylon::TLType::TLTypeUSB)
//            {
//                transportProtocol = TransportProtocol::USB;
//                return true;
//            }
//            if (camType == Pylon::TLType::TLTypeGigE)
//            {
//                transportProtocol = TransportProtocol::GIGE;
//                return true;
//            }
//            if (camType == Pylon::TLType::TLTypeCXP)
//            {
//                transportProtocol = TransportProtocol::CPX;
//                return true;
//            }
//            if (camType == Pylon::TLType::TLTypeCL)
//            {
//                transportProtocol = TransportProtocol::CAMERA_LINK;
//                return true;
//            }
//            if (camType == Pylon::TLType::TLTypeCustom)
//            {
//                transportProtocol = TransportProtocol::CUSTOM;
//                return true;
//            }
//            if (camType == Pylon::TLType::TLTypeIPCam)
//            {
//                transportProtocol = TransportProtocol::UNKNOWN;
//                return true;
//            }
//            if (camType == Pylon::TLType::TLTypeCamEmu)
//            {
//                transportProtocol = TransportProtocol::UNKNOWN;
//                return true;
//            }
//
//            return false;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getTransportProtocolImpl failed, error was: " << e.what());
//        }
//        catch (const std::exception& e)
//        {
//            LOG_ERROR(getCameraId() << ": getTransportProtocolImpl failed, error was: " << e.what());
//        }
//        catch (...)
//        {
//            LOG_ERROR(getCameraId() << ": getTransportProtocolImpl failed.");
//        }
//
//        return false;
//    }
//
//    bool BaslerCam::getConnectedHostImpl(std::string& value) const
//    {
//        TransportProtocol::Type tp;
//        if (!getTransportProtocolImpl(tp) || tp != TransportProtocol::GIGE)
//        {
//            LOG_ERROR(getCameraId() << ": getConnectedHost is not supported by the camera.");
//            return false;
//        }
//
//        try
//        {
//            value = m_camera.GetDeviceInfo().GetInterface();
//            return true;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(getCameraId() << ": getConnectedHost failed, error was: " << e.what());
//        }
//        catch (const std::exception& e)
//        {
//            LOG_ERROR(getCameraId() << ": getConnectedHost failed, error was: " << e.what());
//        }
//        catch (...)
//        {
//            LOG_ERROR(getCameraId() << ": getConnectedHost failed.");
//        }
//
//        return false;
//    }
//
//    bool BaslerCam::enableStaticIpImpl(const std::string& ipAddress, const std::string& subnetMask, const std::string& defaultGateway)
//    {
//        return broadcastIpConfiguration(false, ipAddress, subnetMask, defaultGateway);
//    }
//
//    bool BaslerCam::enableDhcpIpImpl()
//    {
//        return broadcastIpConfiguration(true, "0", "0", "0");
//    }
//
//    bool BaslerCam::broadcastIpConfiguration(bool enableDhcp, const std::string& ipAddress, const std::string& subnetMask, const std::string& defaultGateway)
//    {
//        // Stop aquiring images discard current frames
//        stopCamera();
//
//        // Close camera
//        closeImpl();
//
//        try
//        {
//            // Create GigE transport layer.
//            CTlFactory& tlFactory = CTlFactory::GetInstance();
//
//            auto deleter = [&](ITransportLayer* pTl) { tlFactory.ReleaseTl(pTl); };
//            std::unique_ptr<ITransportLayer, decltype(deleter)> uniqueTl(tlFactory.CreateTl(BaslerGigEDeviceClass), deleter);
//            if (uniqueTl == nullptr)
//            {
//                LOG_ERROR("No GigE transport layer installed. Verify GigE drivers are installed.");
//                return false;
//            }
//
//            // Current ip settings
//            std::string currentIpAddress;
//            std::string currentSubnetMask;
//            std::string currentDefaultGateway;
//            bool isDhcpEnabled = false;
//
//            Pylon::String_t macAddress = "";
//            Pylon::String_t userDefinedName = "";
//
//            // Find all devices
//            Pylon::DeviceInfoList_t devices;
//
//            IGigETransportLayer* tlGigE = dynamic_cast<IGigETransportLayer*>(uniqueTl.get());
//            if (tlGigE == nullptr)
//            {
//                LOG_ERROR("Failed to cast transport layer to GigE.");
//                return false;
//            }
//
//            tlGigE->EnumerateAllDevices(devices); // Enumerates all GigE devices
//
//            // Sanity check if method is called for non-GigE camera
//            if (devices.size() == 0)
//            {
//                LOG_INFO("No GigE devices found to set ip configuration on.");
//                return false;
//            }
//
//            // Get cameras address mac and user defined name   
//            // Iterate over all cameras
//            CInstantCameraArray cameras(devices.size());
//            int cameraIndex = -1;
//            for (size_t i = 0; i < cameras.GetSize(); ++i)
//            {
//                cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
//
//                // Match device and selected cameraInfo via serial number
//                if (strcmp(getCameraInfo().serial.c_str(), cameras[i].GetDeviceInfo().GetSerialNumber().c_str()) == 0)
//                {
//                    cameraIndex = static_cast<int>(i);
//                    currentIpAddress = devices[i].GetIpAddress();
//                    currentSubnetMask = devices[i].GetSubnetMask();
//                    currentDefaultGateway = devices[i].GetDefaultGateway();
//                    isDhcpEnabled = devices[i].IsDhcpActive();
//
//                    macAddress = devices[i].GetMacAddress();
//                    userDefinedName = devices[i].GetUserDefinedName();
//                }
//            }
//
//            // Verify requested camera is connected and could be matched to specified CameraInfos
//            if (cameraIndex == -1)
//            {
//                LOG_ERROR("Failed to set ip configuration on camera " << getCameraInfo().vendorName << ", " << getCameraInfo().productName << ", " << getCameraInfo().serial << ". Matching device not connected.");
//                return false;
//            }
//
//            // If provided settings are already set, do nothing
//            if (enableDhcp && isDhcpEnabled)
//            {
//                return true;
//            }
//
//            if (!enableDhcp && !isDhcpEnabled && currentIpAddress == ipAddress && currentSubnetMask == subnetMask && currentDefaultGateway == defaultGateway)
//            {
//                return true;
//            }
//
//            LOG_INFO("Setting ip configuration for camera: " << formatToReadableMacAddress(macAddress));
//            LOG_INFO("Dhcp: " << enableDhcp);
//            LOG_INFO("Ip: " << ipAddress);
//            LOG_INFO("Subnet mask: " << subnetMask);
//            LOG_INFO("Default gateway: " << defaultGateway);
//
//            const bool setOk = tlGigE->BroadcastIpConfiguration(macAddress, !enableDhcp, enableDhcp, ipAddress.c_str(), subnetMask.c_str(), defaultGateway.c_str(), userDefinedName);
//
//            if (setOk)
//            {
//                // Reboot camera network interface
//                LOG_INFO("Rebooting network interface of camera: " << formatToReadableMacAddress(macAddress));
//                tlGigE->RestartIpConfiguration(macAddress);
//            }
//            else
//            {
//                LOG_ERROR("Failed to set ip configuration for camera: " << formatToReadableMacAddress(macAddress));
//
//                // Reboot camera so future broadcasts will be accepted
//                LOG_INFO("Rebooting camera: " << formatToReadableMacAddress(macAddress));
//                cameras[cameraIndex].Open();
//                GenApi::INodeMap& nodeMap = cameras[cameraIndex].GetNodeMap();
//                CCommandParameter DeviceResetCommand(nodeMap, "DeviceReset");
//                DeviceResetCommand.Execute();
//            }
//
//            // Wait for camera to reconnect
//            const int maxRetries = 30; // 30s
//            bool deviceFoundAndSet = false;
//            bool deviceFoundSetFailed = false;
//            for (int i = 0; i < maxRetries; i++)
//            {
//                LOG_INFO("Waiting for camera to reconnect. Wait: " << i + 1 << "/" << maxRetries);
//                Pylon::WaitObject::Sleep(1000); // Wait 1 second
//                tlGigE->EnumerateAllDevices(devices);
//
//                // Search for device with matching mac address
//                for (int dev = 0; dev < devices.size(); dev++)
//                {
//                    if (devices[dev].GetMacAddress() == macAddress)
//                    {
//                        // Check if ip configuration has been applied
//                        if ((devices[dev].IsDhcpActive() && enableDhcp) ||
//                            (!devices[dev].IsDhcpActive() && !enableDhcp && devices[dev].GetIpAddress() == ipAddress.c_str() && devices[dev].GetSubnetMask() == subnetMask.c_str() && devices[dev].GetDefaultGateway() == defaultGateway.c_str()))
//                        {
//                            deviceFoundAndSet = true;
//                        }
//
//                        if (!setOk)
//                        {
//                            deviceFoundSetFailed = true;
//                        }
//                        break;
//                    }
//                }
//
//                if (deviceFoundAndSet || deviceFoundSetFailed)
//                {
//                    // Leave retry loop
//                    break;
//                }
//            }
//
//            if (deviceFoundAndSet)
//            {
//                LOG_INFO("Ip configuration set successfully for camera: " << formatToReadableMacAddress(macAddress));
//            }
//            else
//            {
//                LOG_ERROR("Ip configuration not applied for camera: " << formatToReadableMacAddress(macAddress));
//            }
//
//            return deviceFoundAndSet;
//        }
//        catch (const Pylon::GenericException& e)
//        {
//            LOG_ERROR(e.GetDescription());
//            return false;
//        }
//    }
//
//    std::string BaslerCam::formatToReadableMacAddress(const Pylon::String_t& macAddressPylon)
//    {
//        std::string result;
//        for (size_t i = 0; i < macAddressPylon.length(); i += 2) {
//            if (i > 0) {
//                result += ":";
//            }
//            result += macAddressPylon.substr(i, 2);
//        }
//        return result;
//
//    }
//
//    Camera::ColorMode::Type BaslerCam::cvtPixelFormatEnumStringToColorMode(const std::string& pixelFormatEnumString) const
//    {
//        static const std::unordered_map<std::string, Camera::ColorMode::Type> colorModeMap =
//        {
//            {"Mono8", Camera::ColorMode::MONO8},
//            {"Mono12", Camera::ColorMode::MONO12},
//            {"Mono12p", Camera::ColorMode::MONO12p},
//            {"Mono16", Camera::ColorMode::MONO16},
//        };
//
//        const auto it = colorModeMap.find(pixelFormatEnumString);
//        if (it != colorModeMap.end())
//        {
//            return it->second;
//        }
//        return Camera::ColorMode::UNKNOWN;
//    }

}
#endif 