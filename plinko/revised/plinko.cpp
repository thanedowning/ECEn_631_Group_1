#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>

int fd, n, i;
char buf[128] = "temp text";

bool clickedPoint = false;
cv::Vec3b clickedPointVal;
cv::Point2f point;
int clickCount = 0;

const int boardWidthCm = 46;
int boardWidthPixels = 0;
int pixelPerCm = 0;
int leftPixelThresh = 0;
int rightPixelThresh = 640;
int topPixelThresh = 0;
int bottomPixelThresh = 480;

static void intro_message() {
  std::cout << "Project 2 for ECEn 631: Plinko Ball Catcher.\n";
}

void sendCommand(const char* command) {
  write(fd, command, strlen(command));
  n = read(fd, buf, 64);
  buf[n] = 0;
}

int setupSerial() {
  struct termios toptions;
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY); // open serial port
  printf("fd opened as %i\n", fd);
  usleep(1500000);                      // wait for the Arduino to reboot
  tcgetattr(fd, &toptions);             // get current serial port settings
  cfsetispeed(&toptions, B115200);      // set 115200 baud both ways
  cfsetospeed(&toptions, B115200);
  toptions.c_cflag &= ~PARENB;          // 8 bits, no parity, no stop bits
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  toptions.c_lflag |= ICANON;           // Canonical mode
  tcsetattr(fd, TCSANOW, &toptions);    // commit the serial port settings
  printf("Attempting to communicate with arduino... \n");
  return 0;
}

static void onMouse(int event, int x, int y, int flags, void* param) {
  cv::Mat &src = *((cv::Mat*)param); //cast and deref the param
  if (event == cv::EVENT_LBUTTONDOWN) {
    clickedPoint = true;
    clickedPointVal = src.at<cv::Vec3b>(y,x);
    point = cv::Point(x,y);
    clickCount++;
  }
}

int getBallXPos(cv::Point pt) {
  int xpos = ((pt.x-leftPixelThresh)/pixelPerCm) + 7;
  if(xpos > 53) {
    xpos = 53;
  }
  else if(xpos < 7) {
    xpos = 7;
  }
  return xpos;
}

cv::Point getBallMoments(cv::Mat& hsvFrame, cv::Mat& binFrame,
                         cv::Scalar lowThresh, cv::Scalar highThresh) {
  cv::Size erodeDilateSize  = cv::Size(12,12);
  cv::inRange(hsvFrame, lowThresh, highThresh, binFrame);
  cv::erode(binFrame, binFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                          erodeDilateSize));
  cv::dilate(binFrame, binFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                           erodeDilateSize));
  cv::Moments moments = cv::moments(binFrame,true);
  cv::Point location = cv::Point(moments.m10/moments.m00,
                                 moments.m01/moments.m00);
  cv::cvtColor(binFrame, binFrame, cv::COLOR_GRAY2BGR);
  cv::circle(binFrame, location, 15, cv::Scalar(255,0,0),2, 8, 0);
  return location;
}

void catchBall(cv::Point ballLocation) {
  std::string out = "g";
  int pos = getBallXPos(ballLocation);
  out.append(std::to_string(pos));
  out.append("\n");
  sendCommand(out.c_str());
}

int main(int argc, char** argv) {
  intro_message();

  cv::VideoCapture vid; // open the default camera
  cv::CommandLineParser parser(argc, argv, "{@vidStreamDevice|0|}");
  std::string vidStreamDevice = parser.get<std::string>("@vidStreamDevice");
  if (vidStreamDevice.size() == 1 && isdigit(vidStreamDevice[0]))
    vid.open(vidStreamDevice[0] - '0');
  else
    vid.open(0);
  if (!vid.isOpened()) {
    std::cout << "Image source not found; program terminating.\n";
    return 0;
  }

  vid.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  vid.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  cv::VideoWriter vidout;
  vidout.open("../out/capture.avi", cv::VideoWriter::fourcc('M','P','E','G'),
              30, cv::Size(vid.get(cv::CAP_PROP_FRAME_WIDTH),
              vid.get(cv::CAP_PROP_FRAME_HEIGHT)), 0);

  cv::Mat inFrame, hsvFrame, redBinFrame, greenBinFrame, blueBinFrame;
  cv::Point redLocation, greenLocation, blueLocation;

  cv::Scalar redLowThresh = cv::Scalar(160,80,40);
  cv::Scalar redHighThresh = cv::Scalar(180,255,255);
  cv::Scalar greenLowThresh = cv::Scalar(70,80,40);
  cv::Scalar greenHighThresh = cv::Scalar(85,255,255);
  cv::Scalar blueLowThresh = cv::Scalar(86,80,40);
  cv::Scalar blueHighThresh = cv::Scalar(102,255,255);

  int hPlusMinusThresh = 20;
  int xPixelValPlusMinusThresh = 50;
  int yPixelValPlusMinusThresh = 50;
  int pegRows = 14;
  int pixelsPerPegRows = 0;
  bool calibrated = false;

  cv::namedWindow("Camera Input", 1);
  cv::namedWindow("Blue Input", 1);
  cv::namedWindow("Green Input", 1);
  cv::namedWindow("Red Input", 1);
  cv::setMouseCallback("Camera Input", onMouse, &hsvFrame);

  setupSerial();

  while(!calibrated) {
    vid >> inFrame;
    if(!inFrame.empty()) {
      if(cv::waitKey(10) == 32) return 0;
      cv::imshow("Camera Input", inFrame);
      cv::cvtColor(inFrame, hsvFrame, cv::COLOR_BGR2HSV);
      if(clickedPoint) {
        if(clickCount == 1) {
          redLowThresh[0] = clickedPointVal[0] - hPlusMinusThresh;
          redHighThresh[0] = clickedPointVal[0] + hPlusMinusThresh;
          clickedPoint = false;
        }
        else if(clickCount == 2) {
          greenLowThresh[0] = clickedPointVal[0] - hPlusMinusThresh;
          greenHighThresh[0] = clickedPointVal[0] + hPlusMinusThresh;
          clickedPoint = false;
        }
        else if(clickCount == 3) {
          blueLowThresh[0] = clickedPointVal[0] - hPlusMinusThresh;
          blueHighThresh[0] = clickedPointVal[0] + hPlusMinusThresh;
          clickedPoint = false;
        }
        else if(clickCount == 4) {
          leftPixelThresh = point.x;
          clickedPoint = false;
        }
        else if(clickCount == 5) {
          rightPixelThresh = point.x;
          bottomPixelThresh = point.y;
          clickedPoint = false;
          boardWidthPixels = rightPixelThresh - leftPixelThresh;
          pixelPerCm = boardWidthPixels / boardWidthCm;
        }
        else if(clickCount == 6) {
          topPixelThresh = point.y;
          clickedPoint = false;
          pixelsPerPegRows = (bottomPixelThresh-topPixelThresh)/pegRows;
          calibrated = true;
        }
        std::cout << clickedPointVal << " at " << point << std::endl;
      }
    }
  }

  sendCommand("h\n"); // Home the motor and encoder

  while(true) {
    vid >> inFrame;
    if(!inFrame.empty()) {
      for (int it_x = 0; it_x < inFrame.cols; it_x++) {
        for (int it_y = 0; it_y < inFrame.rows; it_y++) {
          if (it_x < leftPixelThresh || it_x > rightPixelThresh ||
              it_y < topPixelThresh || it_y > bottomPixelThresh) {
              inFrame.at<cv::Vec3b>(cv::Point(it_x,it_y)) = cv::Vec3b(0,0,0);
          }
        }
      }
      cv::cvtColor(inFrame, hsvFrame, cv::COLOR_BGR2HSV);

      redLocation = getBallMoments(hsvFrame, redBinFrame, redLowThresh,
                                   redHighThresh);
      greenLocation = getBallMoments(hsvFrame, greenBinFrame, greenLowThresh,
                                     greenHighThresh);
      blueLocation = getBallMoments(hsvFrame, blueBinFrame, blueLowThresh,
                                    blueHighThresh);

      /*
      if (redLocation.x < 0 || redLocation.x > 640) {
        if (greenLocation.x < 0 || greenLocation.x > 640) {
          if (blueLocation.x < 0 || blueLocation.x > 640) {
            std::cout << "hit ";
            catchBall(cv::Point((boardWidthPixels/2+leftPixelThresh),0));
          }
          else {
            catchBall(blueLocation);
          }
        }
        else {
          catchBall(greenLocation);
        }
      }
      else {
        catchBall(redLocation);
      }
      */
      if((redLocation.y-topPixelThresh) > pixelsPerPegRows*10) {
        catchBall(redLocation);
        std::cout << "red ";
      }
      else if((greenLocation.y-topPixelThresh) > pixelsPerPegRows*10) {
        catchBall(greenLocation);
        std::cout << "green ";
      }
      else if((blueLocation.y-topPixelThresh) > pixelsPerPegRows*10) {
        catchBall(blueLocation);
        std::cout << "blue ";
      }
      else {
        catchBall(cv::Point((boardWidthPixels/2+leftPixelThresh),0));
      }

      cv::imshow("Camera Input", inFrame);
      cv::imshow("Blue Input", blueBinFrame);
      cv::imshow("Green Input", greenBinFrame);
      cv::imshow("Red Input", redBinFrame);
      if(cv::waitKey(10) == 32) return 0;
    }
  }
  return 0;
}
