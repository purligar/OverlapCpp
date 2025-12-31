#pragma once

#include <string>

class CInterlacedParams
{
public:
    // parameters (defaults match python dataclass)
    double angle = 0.0; // rotation angle in degrees
    double warp = 0.0; // warp factor
    int cuton_16 = 16; // minimum grayvalue in the image
    int max_expected_grayvalue = 65535; // 4095 if 12 Bit image
    int max_mean_grayvalue16 = 40000; // skip image16 if mean grayvalue > this value
    int split_blobs_binary_threshold_8 = 10; // 0..255 for 8Bit intermediate binary image
    int count_lines_binary_threshold_8 = 50; // 0..255 for 8Bit intermediate binary image
    int max_iter_dilate = 100; // maximum number of dilate iterations
    int expand_bounding_box_by_num_px = 50; // expand bounding box by this number of pixels
    int max_erode_iter = 10; // maximum number of erode iterations to get min_num_lines
    int min_num_lines = 10; // minimum number of lines to detect orientation
    int num_blobs_expected = 6; // maximum number of blobs expected
    int min_blob_x = 340; // minimum blob size in x direction in pixel
    int min_blob_y = 340; // minimum blob size in y direction in pixel
	double fResizeFactor = 0.25; // for OpenCV windows
    int debug_output = 15; // bit flags for debug output

    CInterlacedParams() = default;

    // Save and load JSON-like file. Implementations are tolerant to extra whitespace and simple formatting.
    bool SaveToJson(const std::string& path) const;
    static CInterlacedParams LoadFromJson(const std::string& path);
};
