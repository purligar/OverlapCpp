#include "stdafx.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "buildHist.h"

void buildHist(const Mat& src, Mat& histImage)
{
    /// Establish the number of bins
    int histSize = 256;

    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 };
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;
    
    Mat color = src.clone();
    auto ch = src.channels();
    if (src.channels() == 1)
        cvtColor(color, color, CV_GRAY2BGR);
    /// Separate the image in 3 places ( B, G and R )
    Mat bgr_planes[3];
    split(color, bgr_planes);
    
    Mat b_hist, g_hist, r_hist;
    
    /// Compute the histograms:
    calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);
    
    // Draw the histograms for B, G and R
    int hist_w = 512; 
    int hist_h = 320;
    int bin_w = cvRound((double)hist_w / histSize);

    histImage = Mat(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    
    /// Draw for each channel
    for (int i = 1; i < histSize; i++)
    {
        line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
            Point(bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
            Scalar(255, 0, 0), 1, 8, 0);
        line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
            Point(bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))),
            Scalar(0, 255, 0), 1, 8, 0);
        line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
            Point(bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))),
            Scalar(0, 0, 255), 1, 8, 0);
    }
    
    /*namedWindow("histogram", CV_WINDOW_AUTOSIZE);
    imshow("histogram", histImage);
    imwrite("histogram.png", histImage);*/
}