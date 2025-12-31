// LBP.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <clocale>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "common.h"

#include <log4cplus/logger.h>
#include <log4cplus/configurator.h>
#include <log4cplus/loggingmacros.h>

#ifdef PYLON
#include "BaslerCam.h"
#endif

#include "OverlapLib/Overlap.h"
#include "OverlapLib/CInterlacedParams.h"

using namespace std;

/**
* @function on_trackbar
* @brief Callback for trackbar
*/
/*
void on_trackbar_thmin(int, void*)
{
    showThresholded(image32f_out);
}*/

static void help()
{
    std::string msg;
    msg += "\nThis program builds the average of frames\n";
    msg += "Usage:\n";
    msg += "./Overlap [image_path_name -- default is ../sample_images/ ] mintreshold maxtreshold\n\n";
    msg += "q : quit\n";

    std::cout << msg;

    // log the same help message
    auto logger = ::log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Overlap"));
    LOG4CPLUS_INFO(logger, msg.c_str());
}

int _tmain(int argc, _TCHAR* argv[])
{
    int ret = 0;

    ::log4cplus::initialize();     ::log4cplus::PropertyConfigurator::doConfigure("log4cplusconfig.ini");
    auto logger(log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Overlap")));
	LOG4CPLUS_INFO(logger, "Application Started");

    const String keys =
        "{help h usage ? |              | print this message   }"
        "{input         |  ./sample_images  | input file or path or cameradevicenum   }"
        "{cam            |              | supported: opencv, basler   }"
        ;

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        help();
        return 0;
    }
    
    if (!parser.check())
    {
        parser.printErrors();
        return -1;
    }
    
    string sInputPath = parser.get<string>("input");
    if (!sInputPath.empty())
        cout << "InputPath: " << sInputPath << "\n";
    else
    {
        cerr << "Error: empty or invalid InputPath ! \n";
        return -2;
    }

	fs::path outpathfolder = fs::path(sInputPath);
	outpathfolder /= "out";
	// create output folder
	fs::create_directories(outpathfolder);
	cout << "output folder: " << outpathfolder.string() << "\n";

    // prepare params json in output folder
    fs::path params_json_path = outpathfolder;
    params_json_path /= "interlaced_params.json";
    CInterlacedParams interlaced_params;
    if (fs::exists(params_json_path)) {
        interlaced_params = CInterlacedParams::LoadFromJson(params_json_path.string());
        cout << "Loaded interlaced params from: " << params_json_path.string() << "\n";
    }
    else {
        // save defaults
        interlaced_params.SaveToJson(params_json_path.string());
        cout << "Saved default interlaced params to: " << params_json_path.string() << "\n";
    }
    COverlap ov(outpathfolder.string(), interlaced_params);

    if (parser.has("cam"))
    { // images from camera
        auto cam = parser.get<string>("cam");
        if (cam.compare("basler") == 0)
        {
            cout << "try open first Basler/Pylon camera" << "\n";
            #ifdef PYLON
                Pylon::BaslerCam cam;
                auto ret = cam.runBaslerPylonCamera(&ot);
            #endif      
        }
        else if (cam.compare("opencv") == 0)
        {
            cout << "try open first OpenCV camera" << "\n";
            try
            {
                VideoCapture cap;
                const int deviceId = 0;
                cap.open(deviceId);
                if (!cap.isOpened())
                {
                    std::cerr << "Capture Device ID " << deviceId << "cannot be opened." << "\n";
                    return -1;
                }

                Mat orig, grey;
                string sFiletext;
                ov.m_bStopAquisition = false;
                do
                {
                    //std::cout << "try capture frame from VideoCapture device" << "\n";
                    // wait for a new frame from camera and store it into 'frame'
                    cap.read(orig);
                    // check if we succeeded
                    if (orig.empty()) {
                        cerr << "ERROR! blank frame grabbed\n";
                        break;
                    }
                    cv::cvtColor(orig, grey, cv::COLOR_BGR2GRAY);
                    //SVS-Vistek 12-bit Mode scaled to 16 Bit
                    //orig /= 16;

                    if (ov.ProcessImage(grey) != 0)
                        break;
                } while (!ov.m_bStopAquisition);

                cap.release();
            }
            catch (cv::Exception& e)
            {
                const char* err_msg = e.what();
                std::cerr << "exception caught: " << err_msg << "\n";
            }


        } // if (cam.compare("opencv") == 0)
    } // if (parser.has("cam"))
    else if (!sInputPath.empty() && fs::is_directory(fs::path(sInputPath)))
    { // images from filelist
        cout << "load images from: " << sInputPath << "\n";
        auto filelist = GetFileList(sInputPath, "interlaced", { "tif", "tiff" });
        size_t	file_pos = 0;

        bool running = true;
        Mat orig;

        string sFiletext;

        while (running)
        {
            auto fn = filelist[file_pos];
            fs::path filep(fn);
            // loop through filelist
            //cout << "open image: " << fn << "\n";
            orig = cv::imread(fn, cv::IMREAD_ANYDEPTH | cv::IMREAD_GRAYSCALE);
            if (orig.empty())                      // Check for invalid input
            {
                std::cerr << "Could not open or find the image:" << fn << "\n";

                if (file_pos > 0)
                {
                    if (file_pos + 1 < filelist.size())
                        file_pos++;
                }
                else
                    break;
            }
            else
            {
                std::cout << "file_pos:" << file_pos << "; image:" << fn << "\n";
                // build base filename
                string base_filename = fs::path(fn).stem().string();
                // skip if output folder for this image already exists
                fs::path image_outdir = fs::path(ov.m_outpathname) / base_filename;
                if (fs::exists(image_outdir)) {
                    cout << "Output directory exists, skipping: " << image_outdir.string() << "\n";
                } else {
                    // create output dir
                    fs::create_directories(image_outdir);
                    // call SplitBlobs
                    auto results = ov.SplitBlobs(orig, base_filename);
                    cout << "SplitBlobs returned " << results.size() << " entries for " << base_filename << "\n";
                }
            }

            std::cout << "press key to proceed...q or ESC for exit, n for next file, N for previous file" << "\n";
            char key = waitKey(10);
            if (key == -1)
                key = 'n';

            // exit on escape
            if (key == 27)
                running = false;

            // to make it a bit interactive, you can increase and decrease the parameters
            switch (key) {
            case 'q': case 'Q':
                running = false;
                break;
            case 'n':
                file_pos++;
                break;
            case 'N':
                --file_pos;
                break;

            default:
                break;
            }
            if (file_pos >= filelist.size())
                running = false;
            if (file_pos < 0)
                file_pos = filelist.size() - 1;
        }
    }

    return ret;
}
