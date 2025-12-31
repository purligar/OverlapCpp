#include "InterlacedGetOffset.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <numeric>
#include <cmath>

// Minimal port of the Python interlaced_get_offset functionality using OpenCV and std
// This implementation performs column/row mean profile, simple cubic interpolation using OpenCV's resize as approximation,
// local extrema detection is implemented naively.

static std::vector<int> ArgRelExtremaGreater(const std::vector<double>& v) {
    std::vector<int> idx;
    for (size_t i = 1; i + 1 < v.size(); ++i) {
        if (v[i] > v[i-1] && v[i] > v[i+1]) idx.push_back((int)i);
    }
    return idx;
}
static std::vector<int> ArgRelExtremaLess(const std::vector<double>& v) {
    std::vector<int> idx;
    for (size_t i = 1; i + 1 < v.size(); ++i) {
        if (v[i] < v[i-1] && v[i] < v[i+1]) idx.push_back((int)i);
    }
    return idx;
}

OffsetResult InterlacedGetOffset(const cv::Mat& image16, const InterlacedGetOffsetParams& params, double resizeFactor)
{
    OffsetResult ret;
    if (image16.empty()) {
        ret.ErrorCode = -1; ret.ErrorString = "empty image"; return ret;
    }
    if (image16.channels() != 1) { ret.ErrorCode = -2; ret.ErrorString = "not grayscale"; return ret; }

    cv::Mat img;
    if (image16.depth() == CV_16U) img = image16;
    else image16.convertTo(img, CV_16U);

    // compute mean along axis
    cv::Mat profile;
    if (params.axis == 1) { // horizontal hatches -> mean per column
        cv::reduce(img, profile, 0, cv::REDUCE_AVG, CV_64F); // 1 x cols
        profile = profile.reshape(1, profile.cols); // make it a column vector
    } else {
        cv::reduce(img, profile, 1, cv::REDUCE_AVG, CV_64F); // rows x 1
    }

    std::vector<double> column_means;
    column_means.reserve(profile.rows);
    for (int i = 0; i < profile.rows; ++i) column_means.push_back(profile.at<double>(i));

    int n = (int)column_means.size();
    if (n < 10) { ret.ErrorCode = -3; ret.ErrorString = "profile too short"; return ret; }

    // upsample by subpix_multip using simple linear interpolation via resize
    int up_n = n * params.subpix_multip;
    cv::Mat orig(profile);
    cv::Mat up;
    cv::resize(orig, up, cv::Size(1, up_n), 0, 0, cv::INTER_CUBIC);
    std::vector<double> y_interp(up_n);
    for (int i = 0; i < up_n; ++i) y_interp[i] = up.at<double>(i);

    double y_max = *std::max_element(y_interp.begin(), y_interp.end());
    double y_min = *std::min_element(y_interp.begin(), y_interp.end());

    auto maxima_idx = ArgRelExtremaGreater(y_interp);
    auto minima_idx = ArgRelExtremaLess(y_interp);

    std::vector<int> filtered_maxima;
    for (int idx : maxima_idx) {
        if (y_interp[idx] > params.maxima_threshold * y_max) filtered_maxima.push_back(idx);
    }

    ret.num_filtered_maxima = (int)filtered_maxima.size();
    if (ret.num_filtered_maxima < params.expected_num_lines) {
        ret.ErrorCode = -3; ret.ErrorString = "not enough maxima"; // continue to try
    }
    if ((ret.num_filtered_maxima % 2) != 0) {
        ret.ErrorCode = -4; ret.ErrorString = "filtered maxima not even";
    }

    if (ret.num_filtered_maxima < 4) { ret.ErrorCode = -5; ret.ErrorString = "too few maxima"; return ret; }

    // create positions vector
    std::vector<double> subpixel_maxima_positions; subpixel_maxima_positions.reserve(filtered_maxima.size());
    for (int v : filtered_maxima) subpixel_maxima_positions.push_back((double)v);

    double mean_distance_px = 0.0;
    {
        std::vector<double> diffs;
        for (size_t i = 2; i < filtered_maxima.size(); i+=2) diffs.push_back(filtered_maxima[i] - filtered_maxima[i-2]);
        if (!diffs.empty()) {
            mean_distance_px = std::accumulate(diffs.begin(), diffs.end(), 0.0) / diffs.size();
        }
    }
    std::vector<double> diffs2;
    for (size_t i = 3; i < filtered_maxima.size(); i+=2) diffs2.push_back(filtered_maxima[i] - filtered_maxima[i-2]);
    double mean_distance_2_px = !diffs2.empty() ? std::accumulate(diffs2.begin(), diffs2.end(), 0.0) / diffs2.size() : mean_distance_px;
    double expected_distance_px = (mean_distance_px + mean_distance_2_px)/2.0;
    if (expected_distance_px < 10.0) { ret.ErrorCode = -2; ret.ErrorString = "expected_distance_px < 10"; return ret; }

    // compute px2mm using hatch distance
    double px2mm = (subpixel_maxima_positions.back() - subpixel_maxima_positions.front()) / ((subpixel_maxima_positions.size()-1) * params.hatch_distance_mm);
    ret.pos_A = 0.0; ret.pos_B = 0.0;
    // average A and B
    double sumA=0, sumB=0; int cntA=0, cntB=0;
    for (size_t i = 0; i < subpixel_maxima_positions.size(); ++i) {
        if ((i%2)==0) { sumA += subpixel_maxima_positions[i]; cntA++; }
        else { sumB += subpixel_maxima_positions[i]; cntB++; }
    }
    if (cntA>0) ret.pos_A = (sumA/cntA)/px2mm;
    if (cntB>0) ret.pos_B = (sumB/cntB)/px2mm;

    // distances in mm
    std::vector<double> distances_mm;
    for (size_t i = 1; i < subpixel_maxima_positions.size(); ++i) distances_mm.push_back((subpixel_maxima_positions[i] - subpixel_maxima_positions[i-1]) / px2mm);
    std::vector<double> a_dists, b_dists;
    for (size_t i = 0; i+1 < distances_mm.size(); ++i) {
        if ((i%2)==0) a_dists.push_back(distances_mm[i]); else b_dists.push_back(distances_mm[i]);
    }
    double mean_distance_a = 0.0, mean_distance_b=0.0;
    double std_a=0.0, std_b=0.0;
    if (!a_dists.empty()) {
        mean_distance_a = std::accumulate(a_dists.begin(), a_dists.end(), 0.0)/a_dists.size();
        double s=0; for (double v: a_dists) s += (v-mean_distance_a)*(v-mean_distance_a); std_a = std::sqrt(s/a_dists.size());
    }
    if (!b_dists.empty()) {
        mean_distance_b = std::accumulate(b_dists.begin(), b_dists.end(), 0.0)/b_dists.size();
        double s=0; for (double v: b_dists) s += (v-mean_distance_b)*(v-mean_distance_b); std_b = std::sqrt(s/b_dists.size());
    }

    ret.offset = (mean_distance_b - mean_distance_a)/2.0;
    ret.stddev = (std_a + std_b)/2.0;

    // Visualization: draw interpolated profile and extrema
    if (params.debug_output & 1) {
        int H = 400;
        int W = up_n;
        cv::Mat vis(H, W, CV_8UC3, cv::Scalar(255,255,255));
        double ymin = y_min, ymax = y_max;
        if (ymax - ymin < 1e-6) ymax = ymin + 1.0;
        // draw profile polyline
        std::vector<cv::Point> pts; pts.reserve(up_n);
        for (int i = 0; i < up_n; ++i) {
            double v = y_interp[i];
            int py = (int)std::round((1.0 - (v - ymin) / (ymax - ymin)) * (H-1));
            pts.emplace_back(i, py);
        }
        cv::polylines(vis, pts, false, cv::Scalar(0,0,0), 1, cv::LINE_AA);
        // mark maxima
        for (size_t k = 0; k < filtered_maxima.size(); ++k) {
            int xi = filtered_maxima[k];
            int yi = (int)std::round((1.0 - (y_interp[xi] - ymin) / (ymax - ymin)) * (H-1));
            cv::circle(vis, cv::Point(xi, yi), 4, cv::Scalar(255,0,0), -1);
        }
        // mark minima
        for (size_t k = 0; k < minima_idx.size(); ++k) {
            int xi = minima_idx[k];
            int yi = (int)std::round((1.0 - (y_interp[xi] - ymin) / (ymax - ymin)) * (H-1));
            cv::circle(vis, cv::Point(xi, yi), 3, cv::Scalar(0,0,255), -1);
        }
        // draw vertical lines for A and B maxima averages (in pixel units)
        if (!subpixel_maxima_positions.empty()) {
            // compute average positions in subpixel coords
            double avgA = 0, avgB = 0; int cA=0, cB=0;
            for (size_t i=0;i<subpixel_maxima_positions.size();++i) {
                if ((i%2)==0) { avgA += subpixel_maxima_positions[i]; ++cA; } else { avgB += subpixel_maxima_positions[i]; ++cB; }
            }
            if (cA) avgA /= cA;
            if (cB) avgB /= cB;
            cv::line(vis, cv::Point((int)std::round(avgA),0), cv::Point((int)std::round(avgA),H-1), cv::Scalar(0,255,0),1);
            cv::line(vis, cv::Point((int)std::round(avgB),0), cv::Point((int)std::round(avgB),H-1), cv::Scalar(0,165,255),1);
        }

        // annotate distances
        for (size_t i = 0; i+1 < subpixel_maxima_positions.size(); ++i) {
            int x1 = (int)std::round(subpixel_maxima_positions[i]);
            int x2 = (int)std::round(subpixel_maxima_positions[i+1]);
            double dist_mm = (subpixel_maxima_positions[i+1] - subpixel_maxima_positions[i]) / px2mm;
            int xm = (x1 + x2)/2;
            cv::putText(vis, cv::format("%.3f", dist_mm), cv::Point(xm, 15 + (int)(10*(i%3))), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(100,100,100),1);
        }

        std::string savefn = params.base_filename + cv::format("filtered_extrema_profile_%d.png", (int)filtered_maxima.size());
        cv::imwrite(savefn, vis);
        if (params.debug_output & 2) {
            cv::Mat show;
            cv::resize(vis, show, cv::Size(), resizeFactor, resizeFactor);
            cv::imshow("profile", show);
            cv::waitKey(params.debug_output & 2 ? 0 : 1);
        }
    }

    return ret;
}
