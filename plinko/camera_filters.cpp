#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include <iostream>
#include <fstream>
#include <ctype.h>
#include <stdio.h>
#include <cstdint>

static void onMouse(int event, int x, int y, int flags, void* param) {
    cv::Mat &src = *((cv::Mat*)param); //cast and deref the param

    if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Vec3b val = src.at<cv::Vec3b>(y,x);
        std::cout << x << " " << y << " val= "<< val << std::endl;
    }
}

static void intro_message() {
  std::cout << "Test of DIY camera filters\n";
}

int main(int argc, char** argv) {
  intro_message();
  cv::VideoCapture vid;

  cv::CommandLineParser parser(argc, argv, "{@vidStreamDevice|0|}");
  std::string vidStreamDevice = parser.get<std::string>("@vidStreamDevice");

  if(vidStreamDevice.size() == 1 && isdigit(vidStreamDevice[0])) {
    vid.open(vidStreamDevice[0] - '0');
  }
  else {
    vid.open(0);
  }

  if(!vid.isOpened()) {
    std::cout << "Image source not found; program terminating.\n";
    return 0;
  }

  cv::namedWindow("camera_filters", 1);

  cv::Mat inFrame, hsvFrame, grayFrame, backFrame, tempFrame, outFrame;
  cv::Mat hsvFrames[3];
  int addFactor;

  setMouseCallback("camera_filters", onMouse, &hsvFrame);

  while(true) {
    vid >> inFrame;
    cv::cvtColor(inFrame, grayFrame, cv::COLOR_BGR2GRAY);
    cv::cvtColor(inFrame, hsvFrame, cv::COLOR_BGR2HSV);
    cv::split(hsvFrame, hsvFrames);
    if(backFrame.empty()) {
      grayFrame.copyTo(backFrame);
    }
    cv::absdiff(grayFrame, backFrame, outFrame);
    cv::add(hsvFrames[0], addFactor, outFrame);
    cv::imshow("camera_filters", inFrame);
    char input = cv::waitKey(10);
    if(input == 27) {
      break;
    }
    else if(input == 32) {
      addFactor++;
    }
  }
}
