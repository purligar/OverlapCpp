#pragma once

#include <opencv2/opencv.hpp>
#include <string>

struct OffsetResult {
    int ErrorCode = 0;
    std::string ErrorString;
    std::string crop_filename;
    int id_blob = 0;
    std::string orientation;
    int num_filtered_maxima = 0;
    double offset = 0.0;
    double stddev = 0.0;
    double pos_A = 0.0;
    double pos_B = 0.0;
    int bbox_x = 0;
    int bbox_y = 0;
    int bbox_cx = 0;
    int bbox_cy = 0;
};

struct InterlacedGetOffsetParams {
    std::string base_filename;
    int axis = 1; // 0 vertical, 1 horizontal
    double maxima_threshold = 0.5;
    int expected_num_lines = 10;
    double hatch_distance_mm = 0.4;
    int subpix_multip = 100;
    int debug_output = 0;
};

// Added optional resizeFactor: when showing debug windows, images are resized by this factor
OffsetResult InterlacedGetOffset(const cv::Mat& image16, const InterlacedGetOffsetParams& params, double resizeFactor = 1.0);
