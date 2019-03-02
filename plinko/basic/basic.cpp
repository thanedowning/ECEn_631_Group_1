#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>

int fd, n, i;
char buf[128] = "temp text";
cv::Point2f point;          // point (xy pixel location) that is clicked
bool measurePoint = false;  // after mouse click, set to true
bool calibrated = false;    // used to stop the calibration loop
cv::Vec3b clickedPointVal;
int clickCount = 0;

void sendCommand(const char* command) {
  printf("Sending Command: %s", command);
  write(fd, command, strlen(command));  // Send byte to trigger Arduino to send
                                        // string back
  n = read(fd, buf, 64);                //Receive string from Arduino
  buf[n] = 0;                           //insert terminating zero in the string
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

static void intro_message() {
  std::cout << "Project 2 for ECEn 631: Plinko Ball Catcher.\n";

}

static void onMouse(int event, int x, int y, int flags, void* param) {
    cv::Mat &src = *((cv::Mat*)param); //cast and deref the param
    measurePoint = true;
    clickCount++;
    if (event == cv::EVENT_LBUTTONDOWN) {
        clickedPointVal = src.at<cv::Vec3b>(y,x);
        std::cout << x << " " << y << " val= "<< clickedPointVal << std::endl;
    }
}

int main(int argc, char** argv) {
  int frameCounter = 0;
  cv::VideoCapture vid; // open the default camera

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

  cv::Vec3b redColor, greenColor, blueColor;
  cv::Mat inFrame, grayFrame, hsvFrame, outFrame, croppedFrame, prevFrame;
  cv::Mat redBinFrame, greenBinFrame, blueBinFrame;
  std::vector<cv::KeyPoint> keypoints;
  int hsvPlusMinusThresh = 10;
  // bool measuredRed = false;   // whether red ball color has been measured with click
  // bool measuredGreen = false; // same for green ball
  // bool measuredBlue = false;  // same for blue ball

  cv::namedWindow("Camera Input", 1);
  cv::namedWindow("Blue Channel", 1);
  cv::namedWindow("Green Channel", 1);
  cv::namedWindow("Red Channel", 1);
  cv::setMouseCallback("Camera Input", onMouse, &hsvFrame);

  //setupSerial();


  // constexpr cv::Point2f topLeft = (57,144);
  // constexpr int

  //sendCommand("h\n"); // Home the motor and encoder
  // Calibrate colors
  while(!calibrated) {
    vid >> inFrame;
    cv::cvtColor(inFrame, hsvFrame, cv::COLOR_BGR2HSV);
    if(!inFrame.empty()) {
      if(measurePoint) {
        if(clickCount < 2) {
          redColor = clickedPointVal;
          // measuredRed = true;
          measurePoint = false;
          std::cout << "Red Calibration: "<< " val= "<< clickedPointVal << std::endl;
        }
        else if(clickCount < 3) {
          greenColor = clickedPointVal;
          // measuredGreen = true;
          measurePoint = false;
          std::cout << "Green Calibration: "<< " val= "<< clickedPointVal << std::endl;
        }
        else if(clickCount < 4) {
          blueColor = clickedPointVal;
          // measuredBlue = true;
          measurePoint = false;
          calibrated = true;
          std::cout << "Blue Calibration: "<< " val= "<< clickedPointVal << std::endl;
        }
      }
      cv::imshow("Camera Input", inFrame);
      cv::waitKey(10);
    }
  }

  // infinite loop
  while(true) {
    frameCounter++;
    vid >> inFrame; // get a new frame from camera
    if(!inFrame.empty()) {

      // ----- START PROJECT CODE  ----- //
      // Read image
      cv::cvtColor(inFrame, hsvFrame, cv::COLOR_BGR2HSV);




      ////// Blob Detection/////
      //cv::Mat im = cv::imread( "blob.jpg", cv::IMREAD_GRAYSCALE );
      for(int i=0; i < 3; i++) {
        cv::cvtColor(inFrame, grayFrame, CV_BGR2GRAY);

        // Setup SimpleBlobDetector parameters.
        cv::SimpleBlobDetector::Params params;

        // Change thresholds
        params.minThreshold = 10;
        params.maxThreshold = 200;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 1500; // pixel area

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.9; //1 is a perfect circle

        // Filter by Convexity
        params.filterByConvexity = false;
        // params.minConvexity = 0.87;

        // Filter by Inertia
        params.filterByInertia = false;
        // params.minInertiaRatio = 0.01;

        // Set up detector with params
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

        // SimpleBlobDetector::create creates a smart pointer.
        // So you need to use arrow ( ->) instead of dot ( . )
        detector->detect(inFrame, keypoints);
      }




      // Detect blobs.

      // detector.detect( inFrame, keypoints);

      // Draw detected blobs as red circles.
      // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
      cv::Mat im_with_keypoints;
      cv::drawKeypoints(inFrame, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

      // Show blobs
      //cv::imshow("keypoints", im_with_keypoints );

      cv::imshow("Camera Input", inFrame);
      cv::imshow("Blue Input", blueBinFrame);
      cv::imshow("Green Input", greenBinFrame);
      cv::imshow("Red Input", redBinFrame);

      // enter a value greater than 0 to break out of the loop
      if(cv::waitKey(10) >= 0) break;

      /*
      // Command structure is very simple
      // "h\n" is to home the motor
      // "g<integer range 7 to 53>\n" sends the motor to that position in cm
      // e.g. "g35\n" sends the motor to 35cm from left wall
      if(frameCounter%200==0) {
        sendCommand("g10\n");
      }
      else if(frameCounter%100==0){
        sendCommand("g50\n");
      }
      */

    }
  }
  return 0;
}
