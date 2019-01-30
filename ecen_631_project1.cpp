#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

static void intro_message() {
  std::cout << "Project 1 for ECEn 631: a Chex Mix Classifier.\n";

}

int main(int argc, char** argv) {
  cv::VideoCapture vid;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,100,0.01);
  cv::Size subPixWinSize(10,10), winSize(31,31);

  const int MAX_CORNERS = 1000;
  bool show_normal = true;
  bool show_bin = false;
  bool show_canny = false;
  bool show_corners = false;
  bool show_lines = false;
  bool show_dif = false;
  bool classify_objects = false;

  intro_message();

  cv::CommandLineParser parser(argc, argv, "{@vidStreamDevice|0|}");
  std::string vidStreamDevice = parser.get<std::string>("@vidStreamDevice");

  if (vidStreamDevice.size() == 1 && isdigit(vidStreamDevice[0])) {
    vid.open(vidStreamDevice[0] - '0');
  }
  else {
    vid.open(0);
  }

  if (!vid.isOpened()) {
    std::cout << "Image source not found; program terminating.\n";
    return 0;
  }

  cv::VideoWriter vidout;
  vidout.open("../output/result.avi", cv::VideoWriter::fourcc('M','P','E','G'),
              30, cv::Size(vid.get(cv::CAP_PROP_FRAME_WIDTH),
              vid.get(cv::CAP_PROP_FRAME_HEIGHT)), 0);

  cv::namedWindow("ecen_631_project1", 1);

  cv::Mat inFrame, colorFrame, grayFrame, outFrame, tmpFrame, prevFrame;

  std::vector<cv::Point2f> corners;
  std::vector<cv::Vec4i> lines;
  std::vector<cv::Vec3f> circles;

  int binary_threshold_value = 80;
  int max_BINARY_value = 255;
  double canny_threshold_1 = 15;
  double canny_threshold_2 = 80;

  while (true) {
    vid >> inFrame;

    if (inFrame.empty())
      break;

    cv::flip(inFrame, colorFrame, 1);
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_BGR2GRAY);

    if (show_normal){
      cv::swap(grayFrame, outFrame);
    }

    if (show_bin) {
      threshold(grayFrame, outFrame, binary_threshold_value, max_BINARY_value,
                cv::THRESH_BINARY);
    }

    if (show_canny) {
      cv::Canny	(grayFrame, outFrame, canny_threshold_1, canny_threshold_2, 3,
                 false);
    }

    if (show_corners) {
      cv::goodFeaturesToTrack(grayFrame, corners, MAX_CORNERS, 0.01, 50,
                          cv::Mat(), 3, 3, 0, 0.04);
      cv::cornerSubPix(grayFrame, corners, subPixWinSize, cv::Size(-1,-1),
                       termcrit);
      for(int it = 0; it < corners.size(); it++) {
        cv::circle(grayFrame, corners[it], 7, cv::Scalar(0,0,0), -1, 8);
      }
      cv::swap(grayFrame, outFrame);
    }

    if (show_lines) {
      cv::Canny	(grayFrame, tmpFrame, canny_threshold_1, canny_threshold_2, 3,
                 false);
      cv::HoughLinesP(tmpFrame, lines, 1, CV_PI/180, 100, 120, 10);
      cv::swap(grayFrame, outFrame);
      for(int it = 0; it < lines.size(); it++) {
        cv::line(outFrame, cv::Point(lines[it][0], lines[it][1]),
                 cv::Point(lines[it][2], lines[it][3]), cv::Scalar(0,0,0), 5,
                 cv::LINE_AA);
      }
    }

    if (show_dif) {
      if (prevFrame.empty()) {
        cv::swap(grayFrame, prevFrame);
        continue;
      }
      cv::absdiff(grayFrame, prevFrame, outFrame);
      cv::swap(grayFrame, prevFrame);
    }

    if (classify_objects) {
      GaussianBlur(grayFrame, grayFrame, cv::Size(9, 9), 2, 2);
      threshold(grayFrame, grayFrame, binary_threshold_value, max_BINARY_value,
                cv::THRESH_BINARY);
      HoughCircles(grayFrame, circles, cv::HOUGH_GRADIENT, 1, 100, 60,
                   13, 5, 30);
      cv::swap(outFrame, inFrame);
      for(int jt = 0; jt < circles.size(); jt++) {
        cv::Point center(cvRound(circles[jt][0]),
                         cvRound(circles[jt][1]));
        int radius = cvRound(circles[jt][2]);
        circle(outFrame, center, 3, cv::Scalar(255,255,255), -1, 8, 0 );
        circle(outFrame, center, radius, cv::Scalar(255,255,255), 3, 8, 0 );
      }

    }

    vidout.write(outFrame);
    cv::imshow("ecen_631_project1", outFrame);

    char input = cv::waitKey(10);
    if (input == 27) {
      vidout.release();
      break;
    }
    if (input == 32) {
      show_normal = true;

      show_bin = false;
      show_canny = false;
      show_corners = false;
      show_lines = false;
      show_dif = false;

      classify_objects = false;
    }
    if (input == '1'){
      show_bin = true;

      show_normal = false;
      show_canny = false;
      show_corners = false;
      show_lines = false;
      show_dif = false;
    }
    if (input == '2'){
      show_canny = true;

      show_normal = false;
      show_bin = false;
      show_corners = false;
      show_lines = false;
      show_dif = false;
    }
    if (input == '3'){
      show_corners = true;

      show_normal = false;
      show_bin = false;
      show_canny = false;
      show_lines = false;
      show_dif = false;
    }
    if (input == '4'){
      show_lines = true;

      show_normal = false;
      show_bin = false;
      show_canny = false;
      show_corners = false;
      show_dif = false;
    }
    if (input == '5'){
      show_dif = true;

      show_normal = false;
      show_bin = false;
      show_canny = false;
      show_corners = false;
      show_lines = false;
    }
    if (input == 'f'){
      classify_objects = true;
    }
  }
  return 0;
}
