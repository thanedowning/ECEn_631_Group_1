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
  if (event == cv::EVENT_LBUTTONDOWN) {
    measurePoint = true;
    clickCount++;
    clickedPointVal = src.at<cv::Vec3b>(y,x);
    std::cout << x << " " << y << " val= "<< clickedPointVal << std::endl;
    std::cout << "click count" << clickCount << std::endl;
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
  cv::Mat inFrame, grayFrame, hsvFrame, outFrame, croppedFrame, prevFrame, mask;
  cv::Mat redBinFrame, greenBinFrame, blueBinFrame;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat im_with_keypoints;

  /*
        # define range of blue color in HSV
          lower_green = np.array([70,80,0])
          upper_green  = np.array([85,255,255])

          lower_blue = np.array([86,170,0])
          upper_blue = np.array([102,255,255])

          lower_red = np.array([160,100,0])
          upper_red = np.array([180,255,255])*/
  // Thresholds
  int hsvPlusMinusThresh = 5;
  int redLowH = 160;
  int redHighH = 180;
  int greenLowH = 70;
  int greenHighH = 85;
  int blueLowH = 86;
  int blueHighH = 102;
  int lowSat = 80;
  int highSat = 255;
  int lowVal = 40;
  int highVal = 255;

  // Name windows
  cv::namedWindow("Camera Input", 1);
  cv::namedWindow("Blue Input", 1);
  cv::namedWindow("Green Input", 1);
  cv::namedWindow("Red Input", 1);
  cv::setMouseCallback("Camera Input", onMouse, &hsvFrame);

  ////// Blob Detection Parameters/////
  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;
  // Change thresholds
  params.minThreshold = 0;
  params.maxThreshold = 255;
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 20;
  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0.2;
  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.5;
  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.8;
  // Set up detector with params
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

  //setupSerial();

  cv::Point2i topLeft = cv::Point2i(57,144);
  int boardPixelwidth = 332;
  int boardPixelheight = 390;

  // Erode Dilate Size
  cv::Size erodeDilateSize  = cv::Size(1,1);
  //sendCommand("h\n"); // Home the motor and encoder
  // Calibrate colors


  while(!calibrated) {
    vid >> inFrame;
    cv::cvtColor(inFrame, hsvFrame, cv::COLOR_BGR2HSV);
    if(!inFrame.empty()) {
      if(measurePoint) {
        if(clickCount < 2) {
          redColor = clickedPointVal;
          redLowH = redColor[0] - hsvPlusMinusThresh;
          redHighH = redColor[0] + hsvPlusMinusThresh;
          // measuredRed = true;
          measurePoint = false;
          std::cout << "Red Calibration: "<< " val= "<< clickedPointVal << std::endl;

        }
        else if(clickCount < 3) {
          greenColor = clickedPointVal;
          // measuredGreen = true;
          greenLowH = greenColor[0] - hsvPlusMinusThresh;
          greenHighH = greenColor[0] + hsvPlusMinusThresh;
          measurePoint = false;
          std::cout << "Green Calibration: "<< " val= "<< clickedPointVal << std::endl;
        }
        else if(clickCount < 4) {
          blueColor = clickedPointVal;
          // measuredBlue = true;
          blueLowH = blueColor[0] - hsvPlusMinusThresh;
          blueHighH = blueColor[0] + hsvPlusMinusThresh;
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

      //Blue Balls
      cv::inRange(hsvFrame, cv::Scalar(blueLowH,lowSat,lowVal),
                  cv::Scalar(blueHighH,highSat,highVal), blueBinFrame);
      cv::erode(blueBinFrame, blueBinFrame, cv::getStructuringElement(
                cv::MORPH_ELLIPSE, erodeDilateSize));
      cv::dilate(blueBinFrame, blueBinFrame, cv::getStructuringElement(
                 cv::MORPH_ELLIPSE, erodeDilateSize));

      //Green Balls
      cv::inRange(hsvFrame, cv::Scalar(greenLowH,lowSat,lowVal),
                  cv::Scalar(greenHighH,highSat,highVal), greenBinFrame);
      cv::erode(greenBinFrame, greenBinFrame, cv::getStructuringElement(
                cv::MORPH_ELLIPSE, erodeDilateSize));
      cv::dilate(greenBinFrame, greenBinFrame, cv::getStructuringElement(
                 cv::MORPH_ELLIPSE, erodeDilateSize));


      //Red Balls
      cv::inRange(hsvFrame, cv::Scalar(redLowH,lowSat,lowVal),
                  cv::Scalar(redHighH,highSat,highVal), redBinFrame);
      cv::erode(redBinFrame, redBinFrame, cv::getStructuringElement(
                cv::MORPH_ELLIPSE, erodeDilateSize));
      cv::dilate(redBinFrame, redBinFrame, cv::getStructuringElement(
                 cv::MORPH_ELLIPSE, erodeDilateSize));

  /*
      for(int i = 0; i < 3; i++) {

        // Blue Balls
        if(i == 0){
          cv::inRange(hsvFrame, cv::Scalar(blueLowH,lowSat,lowVal),
                      cv::Scalar(blueHighH,highSat,highVal), blueBinFrame);
          mask = blueBinFrame;
        }

        //Green Balls
        else if(i == 1){
          cv::inRange(hsvFrame, cv::Scalar(greenLowH,lowSat,lowVal),
                      cv::Scalar(greenHighH,highSat,highVal), greenBinFrame);
          mask = greenBinFrame;
        }

        //Red Balls
        else{
          cv::inRange(hsvFrame, cv::Scalar(redLowH,lowSat,lowVal),
                      cv::Scalar(redHighH,highSat,highVal), redBinFrame);
          mask = redBinFrame;
        }

        // erode out the noise
    		cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8)));

    		// dilate again to fill in holes
    		cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8)));

        // SimpleBlobDetector::create creates a smart pointer.
        detector->detect(mask, keypoints);

        cv::drawKeypoints(inFrame, keypoints, im_with_keypoints,
          cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        double maxBlob = 0;
        for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
           std::cout << "size of blob is: " << blobIterator->size << std::endl;
           std::cout << "point is at: " << blobIterator->pt.x << " " << blobIterator->pt.y << std::endl;
        }

        // Blue Balls
        if(i == 0){
          cv::drawKeypoints(blueBinFrame, keypoints, blueBinFrame,
            cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        //Green Balls
        }else if(i == 1){
          cv::drawKeypoints(greenBinFrame, keypoints, drawFrame,
            cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        //Red Balls
        }else{
          cv::drawKeypoints(redBinFrame, keypoints, drawFrame,
            cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        }
      }

      */


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
