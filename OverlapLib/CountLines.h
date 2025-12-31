#pragma once

#include <opencv2/opencv.hpp>

struct CountLinesResult {
    int horizontal_lines = 0;
    int vertical_lines = 0;
};

// image: 16-bit single channel
CountLinesResult CountLines(const cv::Mat& image16, const std::string& base_filename, int id_blob, int min_num_lines, int max_erode_iter, int binary_threshold, int debug_output, double resizeFactor = 1.0);
