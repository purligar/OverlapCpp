#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::MatND showHistogram32FC1(const cv::Mat1f& src, float range_min = 0., float range_max = 65535.0, bool logarithmic = true);
