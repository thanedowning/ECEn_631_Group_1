#include <iostream>
#include <ctype.h>
#include <string>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

static void intro_message() {
  std::cout << "Project 2 for ECEn 631: Plinko Ball Catcher.\n";

}

int main(int argc, char** argv) {
  cv::VideoCapture vid;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,
                            100,0.01);
  cv::Size subPixWinSize(10,10), winSize(31,31);

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
  vidout.open("../out/result.avi", cv::VideoWriter::fourcc('M','P','E','G'),
              30, cv::Size(vid.get(cv::CAP_PROP_FRAME_WIDTH),
              vid.get(cv::CAP_PROP_FRAME_HEIGHT)), 0);

  cv::namedWindow("plinko", 1);

  cv::Mat inFrame, colorFrame, grayFrame, outFrame, tmpFrame, prevFrame;

  while (true) {
    vid >> inFrame;

    if (inFrame.empty())
      break;

    cv::flip(inFrame, colorFrame, 1);
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_BGR2GRAY);

    //vidout.write(outFrame);
    cv::imshow("plinko", outFrame);

    char input = cv::waitKey(10);
    if (input == 27) {
      vidout.release();
      break;
    }
    if (input == 32) {

    }
  }
  return 0;
}
