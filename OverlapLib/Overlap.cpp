#include <tchar.h>
#include <iostream>


#include <filesystem>

#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>


#include <fmt/core.h>

#include "Overlap.h"

using namespace std;

namespace fs = std::filesystem;

#include "buildHist.h"
#include "histogram.h"

void MatType(Mat inputMat)
{
    int inttype = inputMat.type();

    string r, a;
    uchar depth = inttype & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (inttype >> CV_CN_SHIFT);
    switch (depth) {
    case CV_8U:  r = "8U";   a = "Mat.at<uchar>(y,x)"; break;
    case CV_8S:  r = "8S";   a = "Mat.at<schar>(y,x)"; break;
    case CV_16U: r = "16U";  a = "Mat.at<ushort>(y,x)"; break;
    case CV_16S: r = "16S";  a = "Mat.at<short>(y,x)"; break;
    case CV_32S: r = "32S";  a = "Mat.at<int>(y,x)"; break;
    case CV_32F: r = "32F";  a = "Mat.at<float>(y,x)"; break;
    case CV_64F: r = "64F";  a = "Mat.at<double>(y,x)"; break;
    default:     r = "User"; a = "Mat.at<UKNOWN>(y,x)"; break;
    }
    r += "C";
    r += (chans + '0');
    std::cout << "Mat is of type " << r << " and should be accessed with " << a << "\n";

}

void printstat(ostream & os, const cv::Mat& src, const std::string& name)
{
    // info min max
    double min;
    double max;
    cv::Point minp, maxp;
    cv::minMaxLoc(src, &min, &max, &minp, &maxp);
    cv::Scalar mean, stddev;
    cv::meanStdDev(src, mean, stddev);
    os << name
        << ": min = " << min << ", max = " << max
        << ", maxp = (" << maxp.x << "," << maxp.y << ")"
        << ", Mean = " << mean[0] 
        << ", StdDev = " << stddev[0]
        << ", minmaxdif = " << max - min << "\n";
}

COverlap::COverlap(const string& outpathname, const double fResizeFactor, const double& minth, const double& maxth)
    :
    m_outpathname(outpathname),
    m_minth(minth),
    m_maxth(maxth),
    m_fResizeFactor(fResizeFactor),
    m_nMedianBlurKernel(3),
    m_uiAcqDelayms(0),
    m_uiProcessDelayms(10),
    m_bIsVerbose(false),
    m_bShowImages(false)
{
    Reset();
}

void COverlap::Reset()
{
    m_of.close();

    m_stack.clear();
    m_bStopAquisition = false;

    m_uiImagesStored = 0;
    m_uiImagesProcessed = 0;
    //m_stddev_k = 0;

    fs::path p(m_outpathname);   // full file path
    p = fs::absolute(p);
    if (!fs::is_directory(p))
    {
        if (fs::create_directory(p))
            cout << "mkdir \"" << p << "\" successfull !\n";
        else
            cerr << "mkdir \"" << p << "\" failed !\n";
    }

    auto message = fmt::format("minth = {:.3}, maxth = {:.3}", m_minth, m_maxth);

    m_outfilesuffix = fmt::format("-minth_{}-maxth_{}", m_minth, m_maxth);

    std::string fn_log = p.string() + "/" + m_outfilesuffix + ".log.txt";
    m_of.open(fn_log.c_str());
    if (m_of.is_open())
        cout << "logfile is: " << fn_log << "\n";
    m_of << message << endl;

    
}

COverlap::~COverlap()
{
    DoOTPostProcess();

    fmt::print("AcqDelay: {}; ProcDelay: {}\n", m_uiAcqDelayms, m_uiProcessDelayms);

    /*destroyAllWindows(); // blockiert leider
    std::cout << "press any key to proceed..." << "\n";
    char key = waitKey(0);*/
}

int COverlap::AddImage(const Mat& m)
{
    int ret = 0;

    {
        //std::lock_guard<std::mutex> guard(m_stackMutex);
        m_stack.push_back(m);
        m_uiImagesStored++;
        cout << ">> stored images: " << m_uiImagesStored << " [" << m_stack.size() << "]" << "\n";
    }


    // OTThread auslagern in main
    // anstossen mittels: https://en.cppreference.com/w/cpp/thread/condition_variable/wait
    if (m_otthread_mutex.try_lock())
    { // OTThread not running
        m_otthread_mutex.unlock();
        //if (!m_execution_future.valid())
        { // (re)start thread
            try
            {
                m_execution_future = std::async(std::launch::async, [this]() {
                    return OTThread();
                    }
                );
            }
            catch (...)
            {
                cerr << "caught ex [1]";
            }
        }
        /*else
            cerr << "error: m_execution_future.valid()";*/
    }
    if (m_uiAcqDelayms > 0)
        this_thread::sleep_for(chrono::milliseconds(m_uiAcqDelayms));
    return ret;
}

int COverlap::OTThread()
{
    int ret = 0;
    
    m_otthread_mutex.lock();
    
    m_bStopImageProcessing = false;
    auto stacksz = m_stack.size();
    cout << "start OTThread: stacksz: [" << stacksz << "]\n";

    do
    {
        while (m_stack.size() > 0)
        {
            {
                //m_stackMutex.lock();
                if (m_stack.size() > 0)
                {
                    auto my_local_copy = m_stack.front().clone();
                    //m_stackMutex.unlock();
                    //cout << "sz: " << my_local_copy.size() << "\n";
                    //printstat(m_of, my_local_copy, "orig");
                    auto ret = ProcessImage(my_local_copy);
                    m_stack.pop_front();
                    cout << "<< stacksz: [" << m_stack.size() << "] - ";
                    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                else
                    ;//m_stackMutex.unlock();
            }

            if (m_bIsVerbose)
                std::cout << "press q to stop acquisition or ESC for exit, a/A to decrease/increase acquisition delay, p/P for processing delay" << "\n";
            char key = waitKey(m_uiProcessDelayms);
            if (key == 'p')
            {
                if (int(m_uiProcessDelayms) - 10 >= 10)
                {
                    m_uiProcessDelayms -= 10;
                    fmt::print("AcqDelay: {}; ProcDelay: {}\n", m_uiAcqDelayms, m_uiProcessDelayms);
                }
            }
            if (key == 'P')
            {
                m_uiProcessDelayms += 10;
                fmt::print("AcqDelay: {}; ProcDelay: {}\n", m_uiAcqDelayms, m_uiProcessDelayms);
            }
            if (key == 'a')
            {
                if (int(m_uiAcqDelayms) - 10 >= 0)
                {
                    m_uiAcqDelayms -= 10;
                    fmt::print("AcqDelay: {}; ProcDelay: {}\n", m_uiAcqDelayms, m_uiProcessDelayms);
                }
            }
            if (key == 'A')
            {
                m_uiAcqDelayms += 10;
                fmt::print("AcqDelay: {}; ProcDelay: {}\n", m_uiAcqDelayms, m_uiProcessDelayms);
            }

            // stop adding new frames
            if (key == 'q')
            {
                std::cout << "q pressed: stop acquisition of new images !" << "\n";
                m_bStopAquisition = true;
            }

            // exit on escape
            if (key == 27)
            {
                std::cout << "ESC pressed: stop image processing loop !" << "\n";
                m_bStopImageProcessing = true;
                break;
            }
        } // while (m_stack.size() > 0)
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (!m_bStopImageProcessing);  

    cout << "end OTThread " << "\n";
    m_otthread_mutex.unlock();

    return ret;
}


int COverlap::ProcessImage(const cv::Mat& orig)
{
    int ret = 0;

    auto start = std::chrono::steady_clock::now();

    //auto bitdepth = orig.depth();
    //auto ch = orig.channels();

    // initialize sum image
    if (m_uiImagesProcessed == 0)
    {

        destroyAllWindows();
        // windows
        //namedWindow("original", WINDOW_AUTOSIZE);
        namedWindow("sum", WINDOW_AUTOSIZE);
        namedWindow("maximage", WINDOW_AUTOSIZE);

        /*/// Create Trackbars
        char TrackbarName_thmin[50];
        sprintf_s(TrackbarName_thmin, "thmin x %d", slider_thmin_max);

        createTrackbar(TrackbarName_thmin, "filt_threshold", &slider_thmin, slider_thmin_max, on_trackbar_thmin);
        */

        m_of << "initialize sum images: begin" << endl;
        MatType(orig);
        m_maximage1f = Mat(orig.rows, orig.cols, CV_32FC1, cvScalar(0));
        m_sum1d = Mat(orig.rows, orig.cols, CV_64FC1, cvScalar(0.));
        //m_averaged1d = Mat(orig.rows, orig.cols, CV_64FC1, cvScalar(0.));
        /*stddevimage = Mat(orig.rows, orig.cols, CV_64FC1, cvScalar(0.));
        M1f = Mat(orig.rows, orig.cols, CV_64FC1, cvScalar(0.));*/
        m_of << "initialize sum images: end" << endl;
    }
    else
    {
        // check file type&size to be identical
        if (orig.size() != m_maximage1f.size())
        {
            cerr << "Error: diferent image size detected !" << endl;
            return -2;
        }
    }

    ++m_uiImagesProcessed;
    auto sLabel = fmt::format("<< processed images: {}", m_uiImagesProcessed);
    cout << sLabel << "\n";
    m_of << sLabel << endl;

    /*if (m_bIsVerbose)
    {
        printstat(cout, orig, "orig");
        printstat(m_of, orig, "orig");
    }*/

    Mat1f orig1f;
    orig.convertTo(orig1f, CV_32FC1);
    auto bitdepth32 = orig1f.depth();
    auto cols32 = orig1f.channels();
    
    //goto on_end;
    printstat(m_of, orig1f, "orig32");

    {   //low and high cutback threshold not working on 64F
        double  computed_threshold;
        // filter out pixel < minth & pixel > maxth;
        if (m_minth > 0)
            computed_threshold = cv::threshold(orig1f, orig1f, m_minth, 0.0, THRESH_TOZERO);
        if (m_maxth > 0)
            computed_threshold = cv::threshold(orig1f, orig1f, m_maxth, 0.0, THRESH_TOZERO_INV);
        if (m_bIsVerbose)
            printstat(m_of, orig1f, "orig32 thresholded");
    }
    //goto on_end;
    if (m_nMedianBlurKernel >= 3 && !(m_nMedianBlurKernel % 2 == 0))
    {//filter out "hot pixel"
        if (1)
        {
            cv::medianBlur(orig1f, orig1f, m_nMedianBlurKernel);
            if (m_bIsVerbose)
                printstat(m_of, orig1f, "orig32 medianBlur");
        }
        else
        {
            //This filter does not work inplace.
            auto bilat = orig1f.clone();
            cv::bilateralFilter(orig1f, bilat, m_nMedianBlurKernel, 500, 15);
            if (m_bIsVerbose)
                printstat(m_of, bilat, "bilateralFilter");
            bilat.copyTo(orig1f);
        }
    }

    //goto on_end;

    if (0)
    {   // substract mean darkfield noise
        cv::Scalar mean, stddev;
        cv::meanStdDev(orig1f, mean, stddev);
        if (m_bIsVerbose)
            std::cout << "Mean: " << mean[0] << " StdDev: " << stddev[0] << endl;

        orig1f = orig1f - mean[0];
    }

    if (m_bShowImages)
        auto hist = showHistogram32FC1(orig1f, 0., 4048.);

    // max image
    m_maximage1f = cv::max(orig1f, m_maximage1f);
    
    //goto on_end;

    if (m_bIsVerbose)
    {
        printstat(cout, m_maximage1f, "m_maximage1f");
        printstat(m_of, m_maximage1f, "m_maximage1f");
    }

    if (m_bShowImages)
    {
        // normalize to 0..255
        Mat normalized;
        normalize(m_maximage1f, normalized, 0, 255, NORM_MINMAX, CV_8UC1);
       // show resized normalized
        Mat normalized_resize = normalized;
        if (m_fResizeFactor < 1.0)
        {
            resize(normalized, normalized_resize, Size(), m_fResizeFactor, m_fResizeFactor);

            int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
            double fontScale = 0.7;
            int thickness = 1;
            Point textOrg(10, 20);
            putText(normalized_resize, sLabel, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
        }
        imshow("maximage", normalized_resize);
    }

    ///*
    //public static double StandardDeviation(List<double> valueList)
    //{
    //double M = 0.0;
    //double S = 0.0;
    //int k = 0;
    //foreach (double value in valueList)
    //{
    //k++;
    //double tmpM = M;
    //M += (value - tmpM) / k;
    //S += (value - tmpM) * (value - M);
    //}
    //return Math.Sqrt(S / (k-1));
    //}*/
    //// sdtdev
    //{
    //    m_stddev_k++;
    //    auto tmpM = M1f;
    //    Mat1f tmp = orig32 - tmpM;
    //    M1f += tmp.mul(1.0 / m_stddev_k);
    //    stddevimage += tmp.mul(orig32 - M1f);
    //    printstat(m_of, stddevimage, "stddevimage");
    //}

    // add act. image
    m_sum1d += orig1f;
    if (m_bIsVerbose)
        printstat(m_of, m_sum1d, "m_sum1d: ");
    /*m_averaged1d += orig32;
    if (m_bIsVerbose)
        printstat(m_of, m_averaged1d, "m_averaged1d: ");*/

    if (m_bShowImages)
    {// normalize to 0..255
        Mat normalized = Mat(m_sum1d.rows, m_sum1d.cols, CV_8UC1, cvScalar(0));
        normalize(m_sum1d, normalized, 0, 255, NORM_MINMAX, CV_8UC1);
        {   // show resized normalized
            Mat normalized_resize = normalized;
            if (m_fResizeFactor < 1.0)
            {
                resize(normalized, normalized_resize, Size(), m_fResizeFactor, m_fResizeFactor);

                int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
                double fontScale = 0.7;
                int thickness = 1;
                Point textOrg(10, 20);
                putText(normalized_resize, sLabel, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
            }
            imshow("sum", normalized_resize);
        }
    }

on_end:
    m_of.flush();
           
    auto end = std::chrono::steady_clock::now();
    //if (m_bIsVerbose)
        std::cout << "Elapsed time in milliseconds: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << '\n';

    return ret;
}


void COverlap::DoOTPostProcess()
{
    // wait for ot thread
    //if (m_bIsVerbose)
        std::cout << "wait for OTThread: ->" << "\n";
    if (m_execution_future.valid())
    {
        try
        {
            m_execution_future.get();
            //cout << "m_execution_future successfully completed" << "\n";
        }
        catch (const std::exception& ex)
        {
            cout << "m_execution_future catch block with exception: " << ex.what() << "\n";
        }
    }
    std::cout << "wait for OTThread: <- done" << "\n";

    auto start = std::chrono::steady_clock::now();
    cout << "start DoOTPostProcess() : output path is: " << m_outpathname << "\n";
    if (m_uiImagesProcessed > 0)
    {
        cout << "\n" << "Build sum, averag, mean ... over " << m_uiImagesProcessed << " images" << "\n";

        // save mean image as 16UC1
        {
            m_sum1d /= (double)m_uiImagesProcessed;
            printstat(m_of, m_sum1d, "m_sum1d");
            Mat imageint16 = Mat(m_sum1d.rows, m_sum1d.cols, CV_16UC1);
            m_sum1d.convertTo(imageint16, CV_16UC1);
            printstat(m_of, imageint16, "meanint16");

            //std::string fn = pout.string() + "mean" + outfilesuffix + ".tif";
            auto fn = fmt::format("{}/mean16{}.tif", m_outpathname, m_outfilesuffix);
            m_of << "save image to file: " << fn;
            auto b = imwrite(fn, imageint16);
            m_of << " done !" << "\n";
        }

        //// save stddevimage image as 16UC1
        //{
        //    stddevimage /= (m_stddev_k - 1);
        //    cv::pow(stddevimage, 0.5, stddevimage);

        //    Mat stddevimage16 = Mat(stddevimage.rows, stddevimage.cols, CV_16UC1);
        //    auto bitdepthint16 = stddevimage.depth();
        //    auto colsint16 = stddevimage.channels();
        //    stddevimage.convertTo(stddevimage16, CV_16UC1);
        //    printstat(m_of, stddevimage16, "stddevimage");

        //    /*Mat imageint16 = Mat(sum.rows, sum.cols, CV_16UC1);
        //    sum.convertTo(imageint16, CV_16UC1);
        //    */
        //    //std::string fn = pout.string() + "stddevimage" + outfilesuffix +".tif";
        //    string fn = fmt::format("{}/stddevimage16{}.tif", m_outpathname, m_outfilesuffix);
        //    m_of << "save image to " << fn;
        //    try {
        //        auto b = imwrite(fn, stddevimage16);
        //    }
        //    catch (runtime_error& ex) {
        //        string fn = fmt::format("Exception saving image: {}\n", ex.what());
        //        m_of << fn;
        //    }
        //    m_of << " done !" << "\n";
        //}

        // save max image as 16UC1
        {
            printstat(m_of, m_maximage1f, "m_maximage1f");

            Mat maximage16 = Mat(m_maximage1f.rows, m_maximage1f.cols, CV_16UC1);
            m_maximage1f.convertTo(maximage16, CV_16UC1);
            printstat(m_of, maximage16, "maximage16");

            //std::string fn = pout.string() + "max" + outfilesuffix + ".tif";
            string fn = fmt::format("{}/maximage16{}.tif", m_outpathname, m_outfilesuffix);
            m_of << "save image to " << fn;
            auto b = imwrite(fn, maximage16);
            m_of << " done !" << "\n";

            /*
            fn = boost::str(boost::format("%s/%s-maximage%s.32bin") % pout.string() % pname.stem().string()  % m_outfilesuffix);
            m_of << "save image to " << fn;
            std::ofstream binFile;
            binFile.open(fn, ios::out | ios::binary);
            auto ch = maximage.channels();
            auto d = maximage.depth();
            binFile.write(maximage.ptr<char>(), maximage.rows * maximage.cols * ch * d); // buggy, stuertzt ab ohne erkennbaren Grund
            binFile.close();
            m_of << " done !" << "\n";
            */
        }

        //// show histogram of averaged image
        //{
        //    Mat avg32 = Mat(averaged.rows, averaged.cols, CV_32FC1);
        //    averaged.convertTo(avg32, CV_32FC1);
        //    printstat(m_of, avg32, "avg32");
        //    showHistogram32FC1(avg32);
        //}
        // save averaged image as binary 64bit file
        //if (0)
        //{
        //    //std::string fn = pathname + m_outfilesuffix + ".64bin";
        //    auto fn = fmt::format("{}/averaged{}.64bin", m_outpathname, m_outfilesuffix);
        //    m_of << "save image to " << fn;
        //    std::ofstream binFile;
        //    binFile.open(fn, ios::out | ios::binary);
        //    binFile.write(averaged.ptr<char>(), averaged.rows * averaged.cols * averaged.channels() * averaged.depth());
        //    binFile.close();
        //    m_of << " done !" << "\n";
        //}
        //// save averaged image as 16UC1
        //{
        //    auto avg = averaged;
        //    Mat averagedint16 = Mat(avg.rows, avg.cols, CV_16UC1);

        //    avg.convertTo(averagedint16, CV_16UC1);

        //    printstat(m_of, averagedint16, "averagedint16");

        //    //std::string fn = pathname + m_outfilesuffix + ".tif";
        //    string fn = fmt::format("{}/averagedint16{}.tif", m_outpathname, m_outfilesuffix);
        //    m_of << "save averagedint16 image to " << fn;
        //    auto b = imwrite(fn, averagedint16);

        //}
    }

    auto end = std::chrono::steady_clock::now();
    //if (m_bIsVerbose)
        std::cout << "DoOTPostProcess(): finished ! Elapsed time in milliseconds: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << '\n';
}
