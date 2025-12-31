#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <deque>
#include <future>
#include <mutex>

#include <opencv2/core.hpp> //Any OPENCV3 code

#include "CInterlacedParams.h"
#include "InterlacedGetOffset.h"

using namespace cv;

/// Global Variables
const double cuiminth = 40.; // dark noise below set to 0 
const double cuimaxth = 4093.; // pixel > set to 0 id = 0 ignored

const double cfResizeFactor = 0.25; // for OpenCV windows

class COverlap
{
public: 
    // reporting
    int        m_iDebugLevel;
    std::string  m_outpathname;
    std::string  m_outfilesuffix;

    // ot parameters, image processing
    double  m_minth, m_maxth;
    int     m_nMedianBlurKernel;

    uint64_t    m_uiImagesProcessed;
    bool    m_bStopAquisition; // to stop adding new images to image stack

    //visualization
    bool    m_bShowImages;
    double  m_fResizeFactor;

    // interlaced processing params (moved here from function arg)
    CInterlacedParams m_params;

private:
    //Mat1f M1f, stddevimage;

    // logging
    std::ofstream   m_of;

public:
    COverlap(const std::string & outpathname, const CInterlacedParams& params, const double fResizeFactor = cfResizeFactor);
    ~COverlap();
    void Reset();
    /*! 
    * \brief       process new image and build 16bit average , max, mean images log file in outpathfolder
    * \returns     0 if no error
    */
    int ProcessImage(const cv::Mat& src);

    // Split blobs (ported from Python interlaced_split_blobs)
    // returns vector of OffsetResult containing offsets and metadata for each crop
    std::vector<OffsetResult> SplitBlobs(const cv::Mat& image, const std::string& base_filename);

private:
   
    
};
