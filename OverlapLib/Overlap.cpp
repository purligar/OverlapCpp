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
#include "CInterlacedParams.h"
#include "CountLines.h"
#include "InterlacedGetOffset.h"

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

COverlap::COverlap(const string& outpathname, const CInterlacedParams& params, const double fResizeFactor)
    :
    m_outpathname(outpathname),
    m_fResizeFactor(fResizeFactor),
    m_params(params),
    m_iDebugLevel(0),
    m_bShowImages(false),
    m_minth(cuiminth),
    m_maxth(cuimaxth),
    m_nMedianBlurKernel(3)
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

    std::string fn_log = p.string() + "/" + ".log.txt";
    m_of.open(fn_log.c_str());
    if (m_of.is_open())
        cout << "logfile is: " << fn_log << "\n";
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

std::vector<OffsetResult> COverlap::SplitBlobs(const cv::Mat& image_in, const std::string& base_filename)
{
    const CInterlacedParams& params = m_params;

    // helper to optionally show window and block when requested
    auto show_if_needed = [&](const std::string& winname, const cv::Mat& mat){
        if (params.debug_output & 2) {
            cv::Mat show;
            if (m_fResizeFactor != 1.0) cv::resize(mat, show, cv::Size(), m_fResizeFactor, m_fResizeFactor);
            else show = mat;
            cv::namedWindow(winname, cv::WINDOW_AUTOSIZE);
            cv::imshow(winname, show);
            cv::waitKey(0);
            cv::destroyWindow(winname);
        }
    };

    std::vector<OffsetResult> results;

    auto log_info = [&](const std::string& s) {
        std::cout << s << std::endl;
        if (m_of.is_open()) m_of << s << std::endl;
    };
    auto log_err = [&](const std::string& s) {
        std::cerr << s << std::endl;
        if (m_of.is_open()) m_of << s << std::endl;
    };

    if (image_in.empty()) {
        log_err("SplitBlobs: input image empty");
        return results;
    }

    cv::Mat image = image_in.clone();
    int h = image.rows, w = image.cols;

    // rotation
    if (std::abs(params.angle) > 1e-8) {
        cv::Point2f center(w/2.0f, h/2.0f);
        cv::Mat M = cv::getRotationMatrix2D(center, params.angle, 1.0);
        cv::warpAffine(image, image, M, image.size(), cv::INTER_LINEAR);
        if (params.debug_output & 2) show_if_needed("rotated", image);
    }

    // warp (simple perspective example if requested)
    if (std::abs(params.warp) > 1e-8) {
        std::vector<cv::Point2f> src = { {0,0}, {(float)w-1,0}, {(float)w-1,(float)h-1}, {0,(float)h-1} };
        std::vector<cv::Point2f> dst = { {0,0}, {(float)w-800,0}, {(float)w-1,(float)h-1}, {0,(float)h-1} };
        cv::Mat mat = cv::getPerspectiveTransform(src, dst);
        cv::Mat warped;
        cv::warpPerspective(image, warped, mat, image.size());
        if (params.debug_output & 1) cv::imwrite(base_filename + "_warped.tiff", warped);
        if (params.debug_output & 2) show_if_needed("warped", warped);
        image = warped;
    }

    // ensure single channel 16-bit
    if (image.channels() != 1) {
        log_err("SplitBlobs: image is not grayscale");
        return results;
    }
    cv::Mat image16;
    if (image.depth() == CV_16U) {
        image16 = image;
    } else if (image.depth() == CV_8U) {
        image.convertTo(image16, CV_16U, 256.0); // <<8
    } else {
        image.convertTo(image16, CV_16U);
    }

    double minVal, maxVal;
    cv::minMaxLoc(image16, &minVal, &maxVal);
    double meanVal = cv::mean(image16)[0];
    log_info(cv::format("image: min_gray_val:%.0f; max_gray_val: %.0f; mean_gray_val: %.0f", minVal, maxVal, meanVal));
    if (meanVal > params.max_mean_grayvalue16) {
        log_err("SplitBlobs: mean too large, skipping");
        return results;
    }

    cv::Mat thr;
    cv::threshold(image16, thr, params.cuton_16, params.max_expected_grayvalue, cv::THRESH_TOZERO);

    if (params.debug_output & 2) {
        cv::Mat tmp; cv::normalize(thr, tmp, 0, 65535, cv::NORM_MINMAX); tmp.convertTo(tmp, CV_8U, 255.0/65535.0);
        show_if_needed("after_cuton", tmp);
    }

    if (params.debug_output & 1) {
        cv::Mat tmp; cv::normalize(thr, tmp, 0, 65535, cv::NORM_MINMAX); tmp.convertTo(tmp, CV_8U, 255.0/65535.0);
        cv::imwrite(base_filename + "_after_cuton.tiff", tmp);
    }

    cv::Mat image8;
    thr.convertTo(image8, CV_8U, 255.0/65535.0);
    cv::Mat image8_rgb;
    cv::cvtColor(image8, image8_rgb, cv::COLOR_GRAY2BGR);

    cv::Mat image8_binary;
    cv::threshold(image8, image8_binary, params.split_blobs_binary_threshold_8, 255, cv::THRESH_BINARY);

    if (params.debug_output & 2) {
        show_if_needed("binary", image8_binary);
    }
    if (params.debug_output & 1) cv::imwrite(base_filename + "_binary.tiff", image8_binary);

    // pad
    cv::Mat image_padded;
    int pad = 10;
    cv::copyMakeBorder(image8_binary, image_padded, pad, pad, pad, pad, cv::BORDER_CONSTANT, cv::Scalar(0));

    cv::Mat image_dilated = image_padded.clone();
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));

    std::vector<std::vector<cv::Point>> contours;
    int dilate_iter = 0;
    for (dilate_iter = 0; dilate_iter < params.max_iter_dilate; ++dilate_iter) {
        if (dilate_iter > 0) cv::dilate(image_padded, image_dilated, kernel, cv::Point(-1,-1), dilate_iter);
        std::vector<std::vector<cv::Point>> cs;
        cv::findContours(image_dilated, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> filtered;
        for (auto &c: cs) {
            double area = cv::contourArea(c);
            if (area >= params.min_blob_x * params.min_blob_y) filtered.push_back(c);
        }
        contours = filtered;
        if ((int)contours.size() >= params.num_blobs_expected) break;
        if (dilate_iter == 0 && (int)contours.size() < params.num_blobs_expected) {
            log_err(cv::format("SplitBlobs: initial contours %d < expected %d, try lower threshold", (int)contours.size(), params.num_blobs_expected));
            break;
        }
        if (params.debug_output & 5) { // bit 0x1 or 0x4 or 0x5 used for intermediate visualization/log
            cv::Mat vis; cv::cvtColor(image_dilated, vis, cv::COLOR_GRAY2BGR);
            cv::drawContours(vis, cs, -1, cv::Scalar(0,0,255), 2);
            if (params.debug_output & 1) cv::imwrite(base_filename + cv::format("_dilate_iter_%d.tiff", dilate_iter), vis);
            if (params.debug_output & 2) {
                show_if_needed("dilate_iter", vis);
            }
        }
    }

    log_info(cv::format("SplitBlobs: contours found: %d after dilate_iter=%d", (int)contours.size(), dilate_iter));
    if (contours.empty()) {
        log_err("SplitBlobs: no contours found");
        return results;
    }

    cv::Mat plotimage = image8_rgb.clone();
    cv::drawContours(plotimage, contours, -1, cv::Scalar(0,0,255), 6);
    if (params.debug_output & 2) {
        show_if_needed("contours", plotimage);
    }
    if (params.debug_output & 1) cv::imwrite(base_filename + "_contours.tiff", plotimage);

    for (size_t blob_id = 0; blob_id < contours.size(); ++blob_id) {
        auto &contour = contours[blob_id];
        cv::Rect r = cv::boundingRect(contour);
        int x = r.x, y = r.y, ww = r.width, hh = r.height;
        int expand = params.expand_bounding_box_by_num_px;
        int x_min = std::max(x - expand - pad, 0);
        int y_min = std::max(y - expand - pad, 0);
        int x_max = std::min(x + ww + expand + pad, image16.cols);
        int y_max = std::min(y + hh + expand + pad, image16.rows);
        cv::Rect crop_rect(x_min, y_min, x_max - x_min, y_max - y_min);

        cv::Mat crop = image16(crop_rect);
        double mean_crop = cv::mean(crop)[0];
        log_info(cv::format("mean_crop: %.0f", mean_crop));

        // call CountLines
        CountLinesResult clres = CountLines(crop, base_filename, (int)blob_id, params.min_num_lines, params.max_erode_iter, params.count_lines_binary_threshold_8, params.debug_output, m_fResizeFactor);
        log_info(cv::format("count_lines result: horiz=%d vert=%d", clres.horizontal_lines, clres.vertical_lines));

        int axis = (clres.horizontal_lines >= clres.vertical_lines) ? 1 : 0;
        std::string suffix = (axis == 1) ? "horiz" : "vert";

        // prepare InterlacedGetOffset params
        InterlacedGetOffsetParams igo_params;
        igo_params.base_filename = base_filename + cv::format("_crop_%d_%s.tiff", (int)blob_id, suffix.c_str());
        igo_params.axis = axis;
        igo_params.maxima_threshold = 0.45;
        igo_params.expected_num_lines = params.num_blobs_expected; // best-effort
        igo_params.hatch_distance_mm = 0.4;
        igo_params.subpix_multip = 100;
        igo_params.debug_output = params.debug_output;

        OffsetResult offres = InterlacedGetOffset(crop, igo_params, m_fResizeFactor);
        if (params.debug_output & 4) {
            // print maxima index info
            std::cout << "InterlacedGetOffset debug: num_maxima=" << offres.num_filtered_maxima << " posA=" << offres.pos_A << " posB=" << offres.pos_B << "\n";
        }
        log_info(cv::format("InterlacedGetOffset: ErrorCode=%d offset=%.4f stddev=%.4f num_maxima=%d", offres.ErrorCode, offres.offset, offres.stddev, offres.num_filtered_maxima));

        // save crop image if requested
        if (params.debug_output & 1) {
            cv::Mat saveimg; crop.convertTo(saveimg, CV_8U, 255.0/65535.0);
            cv::imwrite(base_filename + cv::format("_crop_%d_%s.tiff", (int)blob_id, suffix.c_str()), saveimg);
        }
        if (params.debug_output & 2) {
            cv::Mat disp; crop.convertTo(disp, CV_8U, 255.0/65535.0);
            show_if_needed("crop", disp);
        }

        // collect result info
        OffsetResult out = offres;
        out.crop_filename = base_filename + cv::format("_crop_%d_%s.tiff", (int)blob_id, suffix.c_str());
        out.id_blob = (int)blob_id;
        out.orientation = suffix;
        out.bbox_x = x;
        out.bbox_y = y;
        if (cv::moments(contour).m00 != 0) {
            out.bbox_cx = (int)(cv::moments(contour).m10 / cv::moments(contour).m00);
            out.bbox_cy = (int)(cv::moments(contour).m01 / cv::moments(contour).m00);
        }
        results.push_back(out);
    }

    // write CSV results to outpath
    try {
        if (!results.empty()) {
            fs::path outdir = fs::path(m_outpathname) / base_filename;
            fs::create_directories(outdir);
            fs::path csv = outdir / "layer_offset_values.csv";
            std::ofstream ofs;
            bool exists = fs::exists(csv);
            ofs.open(csv, std::ios::app);
            if (!exists) {
                ofs << "layer,LayerErrorCode,ContoursFound,dilate_iter,BlobsWithErrors,blob_id,num_filtered_maxima,x,y,cx,cy,orientation,offset,stddev,pos_A,pos_B,ErrorCode,ErrorString\n";
            }
            for (auto &r : results) {
                ofs << "0,0," << contours.size() << "," << dilate_iter << ",0,"; // dummy layer fields
                ofs << r.id_blob << "," << r.num_filtered_maxima << "," << r.bbox_x << "," << r.bbox_y << "," << r.bbox_cx << "," << r.bbox_cy << "," << r.orientation << "," << r.offset << "," << r.stddev << "," << r.pos_A << "," << r.pos_B << "," << r.ErrorCode << ",\"" << r.ErrorString << "\"\n";
            }
            ofs.close();
            log_info(std::string("Wrote results to: ") + csv.string());
        }
    } catch (...) {
        log_err("Failed to write CSV results");
    }

    return results;
}







