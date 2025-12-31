#include "CountLines.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

CountLinesResult CountLines(const cv::Mat& image16, const std::string& base_filename, int id_blob, int min_num_lines, int max_erode_iter, int binary_threshold, int debug_output, double resizeFactor)
{
    CountLinesResult ret;
    if (image16.empty()) return ret;

    cv::Mat image;
    if (image16.depth() == CV_16U) image = image16;
    else image16.convertTo(image, CV_16U);

    // convert to 8-bit for processing
    cv::Mat image8;
    image.convertTo(image8, CV_8U, 255.0/65535.0);

    cv::Mat image8_binary;
    cv::threshold(image8, image8_binary, binary_threshold, 255, cv::THRESH_BINARY);

    if (debug_output & 1) {
        cv::imwrite(base_filename + "_binary_" + std::to_string(id_blob) + ".png", image8_binary);
    }
    if (debug_output & 2) {
        cv::Mat show;
        if (resizeFactor != 1.0) cv::resize(image8_binary, show, cv::Size(), resizeFactor, resizeFactor);
        else show = image8_binary;
        cv::imshow("count_lines_binary", show);
        cv::waitKey(debug_output & 2 ? 0 : 1);
    }

    std::vector<cv::Vec4i> horizontal_lines;
    std::vector<cv::Vec4i> vertical_lines;

    for (int i = 1; i <= max_erode_iter; ++i) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
        cv::Mat image_eroded;
        cv::erode(image8_binary, image_eroded, kernel, cv::Point(-1,-1), i);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(image_eroded, lines, 1, CV_PI/90, 100, 50, 20);
        horizontal_lines.clear();
        vertical_lines.clear();
        for (auto &ln : lines) {
            int x1 = ln[0], y1 = ln[1], x2 = ln[2], y2 = ln[3];
            int dx = x2 - x1;
            int dy = y2 - y1;
            double angle = std::atan2((double)dy, (double)dx) * 180.0 / CV_PI;
            angle = std::fmod(std::abs(angle), 180.0);
            if (std::abs(angle - 90.0) < 5.0) vertical_lines.push_back(ln);
            else if (std::abs(angle) < 5.0 || std::abs(angle - 180.0) < 5.0) horizontal_lines.push_back(ln);
        }

        if (!horizontal_lines.empty() || !vertical_lines.empty()) {
            std::cout << base_filename << " Iteration " << i << ": horizontal lines: " << horizontal_lines.size() << ", vertical lines: " << vertical_lines.size() << "\n";
        }

        if ((int)horizontal_lines.size() >= min_num_lines || (int)vertical_lines.size() >= min_num_lines) break;

        if (debug_output & 2) {
            cv::Mat show;
            if (resizeFactor != 1.0) cv::resize(image_eroded, show, cv::Size(), resizeFactor, resizeFactor);
            else show = image_eroded;
            cv::imshow("count_lines_eroded", show);
            cv::waitKey(debug_output & 2 ? 0 : 1);
        }
    }

    ret.horizontal_lines = (int)horizontal_lines.size();
    ret.vertical_lines = (int)vertical_lines.size();

    if (debug_output & 1) {
        cv::Mat image8_rgb;
        cv::cvtColor(image8_binary, image8_rgb, cv::COLOR_GRAY2BGR);
        for (auto &ln: vertical_lines) cv::line(image8_rgb, cv::Point(ln[0], ln[1]), cv::Point(ln[2], ln[3]), cv::Scalar(0,255,0), 2);
        for (auto &ln: horizontal_lines) cv::line(image8_rgb, cv::Point(ln[0], ln[1]), cv::Point(ln[2], ln[3]), cv::Scalar(0,0,255), 2);
        cv::imwrite(base_filename + "_lines_" + std::to_string(id_blob) + ".png", image8_rgb);
    }
    if (debug_output & 2) {
        cv::Mat image8_rgb;
        cv::cvtColor(image8_binary, image8_rgb, cv::COLOR_GRAY2BGR);
        for (auto &ln: vertical_lines) cv::line(image8_rgb, cv::Point(ln[0], ln[1]), cv::Point(ln[2], ln[3]), cv::Scalar(0,255,0), 2);
        for (auto &ln: horizontal_lines) cv::line(image8_rgb, cv::Point(ln[0], ln[1]), cv::Point(ln[2], ln[3]), cv::Scalar(0,0,255), 2);
        cv::Mat show;
        if (resizeFactor != 1.0) cv::resize(image8_rgb, show, cv::Size(), resizeFactor, resizeFactor);
        else show = image8_rgb;
        cv::imshow("count_lines_lines", show);
        cv::waitKey(debug_output & 2 ? 0 : 1);
    }

    return ret;
}
