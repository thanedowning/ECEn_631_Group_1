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
bool measuredRed = false;   // whether red ball color has been measured with click
bool measuredGreen = false; // same for green ball
bool measuredBlue = false;  // same for blue ball
bool calibrated = false;    // used to stop the calibration loop

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

static void onMouse(int event, int x, int y, int flags, void* userdata) {
    if(event == cv::EVENT_LBUTTONDOWN) {
        point = cv::Point2f((float)x, (float)y);
        measurePoint = true;
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

  cv::namedWindow("Camera Input", 1);
  cv::setMouseCallback("Camera Input", onMouse, 0);

  setupSerial();

  cv::Vec3b redColor, greenColor, blueColor;
  cv::Mat inFrame, grayFrame, outFrame, tmpFrame, prevFrame;
  // constexpr cv::Point2f topLeft = (240,20);
  // constexpr cv::Point2f topRight = (740,20);
  // constexpr cv::Point2f bottomLeft = (210,670);
  // constexpr cv::Point2f bottomRight = (750,670);

  sendCommand("h\n"); // Home the motor and encoder

  // Calibrate colors
  while(!calibrated) {
    vid >> inFrame;
    if(!inFrame.empty()) {
      if(measurePoint) {
        if(!measuredRed) {
          redColor = inFrame.at<cv::Vec3b>(point);
          measuredRed = true;
          measurePoint = false;
        }
        else if(!measuredGreen) {
          greenColor = inFrame.at<cv::Vec3b>(point);
          measuredGreen = true;
          measurePoint = false;
        }
        else if(!measuredBlue) {
          blueColor = inFrame.at<cv::Vec3b>(point);
          measuredBlue = true;
          measurePoint = false;
          calibrated = true;
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

      // Blob Detection
      Mat im = imread( "blob.jpg", IMREAD_GRAYSCALE );

      // Set up the detector with default parameters.
      cv::SimpleBlobDetector detector;

      // Detect blobs.
      std::vector<cv::KeyPoint> keypoints;
      detector.detect(im, keypoints);

      // Draw detected blobs as red circles.
      // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the
      // circle corresponds to the size of blob
      cv::Mat im_with_keypoints;
      cv::drawKeypoints(im, keypoints, im_with_keypoints, cv::Scalar(0,0,255),
                        DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      cv::imshow("Camera Input", inFrame);

      if(cv::waitKey(10) >= 0) break;

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
    }
  }
  return 0;
}
