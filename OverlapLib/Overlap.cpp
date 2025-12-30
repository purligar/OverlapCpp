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

void printtype(ostream& os, Mat inputMat)
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
    os << "Mat is of type " << r << " and should be accessed with " << a << "\n";

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
    m_iDebugLevel(0),
    m_bShowImages(false)
{
    Reset();
}

void COverlap::Reset()
{
    m_of.close();

    m_bStopAquisition = false;
    m_uiImagesProcessed = 0;

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
    //fmt::print("AcqDelay: {}; ProcDelay: {}\n", m_uiAcqDelayms, m_uiProcessDelayms);

    /*destroyAllWindows(); // blockiert leider
    std::cout << "press any key to proceed..." << "\n";
    char key = waitKey(0);*/
}

int COverlap::ProcessImage(const cv::Mat& orig)
{
    int ret = 0;
    std::stringstream ss;
    auto start = std::chrono::steady_clock::now();

    ++m_uiImagesProcessed;
    auto sLabel = fmt::format("<< processed images: {}", m_uiImagesProcessed);
    cout << sLabel << "\n";
    printtype(ss, orig);
	cout << ss.str(); // print type to console
	ss.str(std::string()); // clear
	printstat(ss, orig, "orig");
    cout << ss.str();
    ss.str(std::string()); // clear

    //destroyAllWindows();
    //// windows
    ////namedWindow("original", WINDOW_AUTOSIZE);
    //namedWindow("sum", WINDOW_AUTOSIZE);
    //namedWindow("maximage", WINDOW_AUTOSIZE);

    ///*/// Create Trackbars
    //char TrackbarName_thmin[50];
    //sprintf_s(TrackbarName_thmin, "thmin x %d", slider_thmin_max);

    //createTrackbar(TrackbarName_thmin, "filt_threshold", &slider_thmin, slider_thmin_max, on_trackbar_thmin);
    //*/


    //

    ///*if (m_bIsVerbose)
    //{
    //    printstat(cout, orig, "orig");
    //    printstat(m_of, orig, "orig");
    //}*/

    //Mat1f orig1f;
    //orig.convertTo(orig1f, CV_32FC1);
    //auto bitdepth32 = orig1f.depth();
    //auto cols32 = orig1f.channels();
    //
    ////goto on_end;
    //printstat(m_of, orig1f, "orig32");

    //{   //low and high cutback threshold not working on 64F
    //    double  computed_threshold;
    //    // filter out pixel < minth & pixel > maxth;
    //    if (m_minth > 0)
    //        computed_threshold = cv::threshold(orig1f, orig1f, m_minth, 0.0, THRESH_TOZERO);
    //    if (m_maxth > 0)
    //        computed_threshold = cv::threshold(orig1f, orig1f, m_maxth, 0.0, THRESH_TOZERO_INV);
    //    if (m_iDebugLevel>0)
    //        printstat(m_of, orig1f, "orig32 thresholded");
    //}
    ////goto on_end;
    //if (m_nMedianBlurKernel >= 3 && !(m_nMedianBlurKernel % 2 == 0))
    //{//filter out "hot pixel"
    //    if (1)
    //    {
    //        cv::medianBlur(orig1f, orig1f, m_nMedianBlurKernel);
    //        if (m_iDebugLevel>0)
    //            printstat(m_of, orig1f, "orig32 medianBlur");
    //    }
    //    else
    //    {
    //        //This filter does not work inplace.
    //        auto bilat = orig1f.clone();
    //        cv::bilateralFilter(orig1f, bilat, m_nMedianBlurKernel, 500, 15);
    //        if (m_iDebugLevel)
    //            printstat(m_of, bilat, "bilateralFilter");
    //        bilat.copyTo(orig1f);
    //    }
    //}

    ////goto on_end;

    //if (0)
    //{   // substract mean darkfield noise
    //    cv::Scalar mean, stddev;
    //    cv::meanStdDev(orig1f, mean, stddev);
    //    if (m_iDebugLevel)
    //        std::cout << "Mean: " << mean[0] << " StdDev: " << stddev[0] << endl;

    //    orig1f = orig1f - mean[0];
    //}

    //if (m_bShowImages)
    //    auto hist = showHistogram32FC1(orig1f, 0., 4048.);

    //// max image
    //m_maximage1f = cv::max(orig1f, m_maximage1f);
    //
    ////goto on_end;

    //if (m_iDebugLevel)
    //{
    //    printstat(cout, m_maximage1f, "m_maximage1f");
    //    printstat(m_of, m_maximage1f, "m_maximage1f");
    //}

    //if (m_bShowImages)
    //{
    //    // normalize to 0..255
    //    Mat normalized;
    //    normalize(m_maximage1f, normalized, 0, 255, NORM_MINMAX, CV_8UC1);
    //   // show resized normalized
    //    Mat normalized_resize = normalized;
    //    if (m_fResizeFactor < 1.0)
    //    {
    //        resize(normalized, normalized_resize, Size(), m_fResizeFactor, m_fResizeFactor);

    //        int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
    //        double fontScale = 0.7;
    //        int thickness = 1;
    //        Point textOrg(10, 20);
    //        putText(normalized_resize, sLabel, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
    //    }
    //    imshow("maximage", normalized_resize);
    //}

    /////*
    ////public static double StandardDeviation(List<double> valueList)
    ////{
    ////double M = 0.0;
    ////double S = 0.0;
    ////int k = 0;
    ////foreach (double value in valueList)
    ////{
    ////k++;
    ////double tmpM = M;
    ////M += (value - tmpM) / k;
    ////S += (value - tmpM) * (value - M);
    ////}
    ////return Math.Sqrt(S / (k-1));
    ////}*/
    ////// sdtdev
    ////{
    ////    m_stddev_k++;
    ////    auto tmpM = M1f;
    ////    Mat1f tmp = orig32 - tmpM;
    ////    M1f += tmp.mul(1.0 / m_stddev_k);
    ////    stddevimage += tmp.mul(orig32 - M1f);
    ////    printstat(m_of, stddevimage, "stddevimage");
    ////}

    //// add act. image
    //m_sum1d += orig1f;
    //if (m_iDebugLevel)
    //    printstat(m_of, m_sum1d, "m_sum1d: ");
    ///*m_averaged1d += orig32;
    //if (m_bIsVerbose)
    //    printstat(m_of, m_averaged1d, "m_averaged1d: ");*/

    //if (m_bShowImages)
    //{// normalize to 0..255
    //    Mat normalized = Mat(m_sum1d.rows, m_sum1d.cols, CV_8UC1, cvScalar(0));
    //    normalize(m_sum1d, normalized, 0, 255, NORM_MINMAX, CV_8UC1);
    //    {   // show resized normalized
    //        Mat normalized_resize = normalized;
    //        if (m_fResizeFactor < 1.0)
    //        {
    //            resize(normalized, normalized_resize, Size(), m_fResizeFactor, m_fResizeFactor);

    //            int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
    //            double fontScale = 0.7;
    //            int thickness = 1;
    //            Point textOrg(10, 20);
    //            putText(normalized_resize, sLabel, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
    //        }
    //        imshow("sum", normalized_resize);
    //    }
    //}

on_end:
    m_of.flush();
           
    auto end = std::chrono::steady_clock::now();
    //if (m_bIsVerbose)
        std::cout << "Elapsed time in milliseconds: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << '\n';

    return ret;
}


