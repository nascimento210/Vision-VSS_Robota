#include <iostream>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include "cvui.h"

void filters(cv::Mat& src, cv::Mat& dst);

void HsvMask(cv::Mat& src, std::vector<float> hsvmin, std::vector<float> hsvmax, cv::Mat& masks);

void tableCorners(int mousex[], int mousey[], int& cont, int mouseStatus);

void perspective(cv::Mat& src, cv::Mat& dst, cv::Mat& transform, bool apply);

void totalMask(std::vector<cv::Mat>&, cv::Mat& finalMask);

void minArea(std::vector<std::vector<cv::Point>>& contorno);

void centers(std::vector<std::vector<cv::Point>>& contorno, std::vector<cv::Point>& poseColor);

void robotPose(std::vector < std::vector<cv::Point>>& teamPose, std::vector<std::vector<cv::Point>>& poseColor);