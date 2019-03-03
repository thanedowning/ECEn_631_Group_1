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
cv::Point2f point;          // point (xy pixel location) that is clicked
bool measurePoint = false;  // after mouse click, set to true
bool calibrated = false;    // used to stop the calibration loop
cv::Vec3b clickedPointVal;
int clickCount = 0;
cv::Point2f leftBinPixel;
cv::Point2f rightBinPixel;
cv::Point2f blueLocation;
cv::Point2f redLocation;
cv::Point2f greenLocation;
double boardWidthPixels = 0;
double boardWidthCm = 46;
double cmPerPixel = 0;


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
    point = cv::Point(x,y);
  }
}

int getBallPos(cv::Point2f pt){
  // Get x position of ball in cm
  int xpos = int((pt.x - leftBinPixel.x) * cmPerPixel) + 7;
  if(xpos > 53) {
    xpos = 53;
    std::cout << "ERROR: Ball position calculated at > 53 cm\n";
  }
  else if(xpos < 7) {
    xpos = 7;
    std::cout << "ERROR: Ball position calculated at < 7 cm\n";
  }
  return xpos;
}

void catchBall(std::string ball){
  std::string cmd;
  int pos = 0;
  std::stringstream stream;

  // catch red ball
  if(ball.compare("red") == 0){
    // Use the global Point2f "redLocation" to command motor
    std::cout << "catch red ball" << std::endl;
    pos = int(getBallPos(redLocation));
  // catch green ball
  }
  else if(ball.compare("green") == 0){
    // Use the global Point2f "greenLocation" to command motor
    std::cout << "catch green ball" << std::endl;
    pos = int(getBallPos(greenLocation));
  // catch blue ball
  }
  else if(ball.compare("blue") == 0){
    // Use the global Point2f "blueLocation" to command motor
    std::cout << "catch blue ball" << std::endl;
    pos = int(getBallPos(blueLocation));
  }
  stream << "g" << pos << "\n";
  cmd = stream.str();
  sendCommand(cmd.c_str());
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

  vid.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  vid.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  cv::VideoWriter vidout;
  vidout.open("../out/result.avi", cv::VideoWriter::fourcc('M','P','E','G'),
              30, cv::Size(vid.get(cv::CAP_PROP_FRAME_WIDTH),
              vid.get(cv::CAP_PROP_FRAME_HEIGHT)), 0);

  cv::Vec3b redColor, greenColor, blueColor;
  cv::Mat inFrame, grayFrame, hsvFrame, outFrame, croppedFrame, prevFrame, mask;
  cv::Mat redBinFrame, greenBinFrame, blueBinFrame;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat im_with_keypoints;
  cv::Moments m;

  int redBall_y, greenBall_y, blueBall_y;
  int yDistThresh = 10;
  int catchWhich = 0;  // 1 for catching green; 2 for catching blue


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
  params.minArea = 10;
  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.2;
  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.5;
  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.8;
  // Set up detector with params
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

  setupSerial();
//typing something 

  cv::Point2i topLeft = cv::Point2i(57,144);
  int boardPixelwidth = 332;
  int boardPixelheight = 390;

  // Erode Dilate Size
  cv::Size erodeDilateSize  = cv::Size(12,12);
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
          std::cout << "Blue Calibration: "<< " val= "<< clickedPointVal << std::endl;
        }
        // left bin calibration
        else if(clickCount < 5) {
          measurePoint = false;
          std::cout << "Left Bin Calibration: " << std::endl;
          std::cout << "x = " << point.x << " y = " << point.y << std::endl;
          leftBinPixel = point;
        }
        // right bin calibration
        else if(clickCount < 6) {
          measurePoint = false;
          std::cout << "Right Bin Calibration: " << std::endl;
          std::cout << "x = " << point.x << " y = " << point.y << std::endl;
          rightBinPixel = point;
          calibrated = true;
          boardWidthPixels = rightBinPixel.x - leftBinPixel.x;
          cmPerPixel = boardWidthCm / boardWidthPixels;
        }
      }
      cv::imshow("Camera Input", inFrame);
      cv::waitKey(10);
    }
  }

  sendCommand("h\n"); // Home the motor and encoder


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

      // USE FOR BLOB DETECTION //
      // detector->detect(blueBinFrame, keypoints);
      // cv::drawKeypoints(inFrame, keypoints, im_with_keypoints,
      // cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      //
      // cv::drawKeypoints(blueBinFrame, keypoints, blueBinFrame,
      //  cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      // double maxBlob = 0;
      // for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); 
      //     blobIterator != keypoints.end(); blobIterator++){
      //   std::cout << "size of blue blob is: " << blobIterator->size << std::endl;
      //   std::cout << "point is at: " << blobIterator->pt.x << " " << blobIterator->pt.y << std::endl;
      // }
      m = cv::moments(blueBinFrame,true);
  		blueLocation = cv::Point(m.m10/m.m00, m.m01/m.m00);

  		// coordinates of centroid
  		// std::cout<< cv::Mat(blueLocation)<< std::flush;

      cv::cvtColor(blueBinFrame, blueBinFrame, cv::COLOR_GRAY2BGR);
  		cv::circle( blueBinFrame, blueLocation,
  		15,
  		cv::Scalar(255,
  		0,0),
  		2, 8, 0 );


      //Green Balls
      cv::inRange(hsvFrame, cv::Scalar(greenLowH,lowSat,lowVal),
                  cv::Scalar(greenHighH,highSat,highVal), greenBinFrame);
      cv::erode(greenBinFrame, greenBinFrame, cv::getStructuringElement(
                cv::MORPH_ELLIPSE, erodeDilateSize));
      cv::dilate(greenBinFrame, greenBinFrame, cv::getStructuringElement(
                 cv::MORPH_ELLIPSE, erodeDilateSize));
      m = cv::moments(greenBinFrame,true);
  		greenLocation = cv::Point(m.m10/m.m00, m.m01/m.m00);


  		// coordinates of centroid
  		// std::cout<< cv::Mat(greenLocation)<< std::flush;

      cv::cvtColor(greenBinFrame, greenBinFrame, cv::COLOR_GRAY2BGR);
  		cv::circle( greenBinFrame, greenLocation,
  		15,
  		cv::Scalar(255,
  		0,0),
  		2, 8, 0 );


      //Red Balls
      cv::inRange(hsvFrame, cv::Scalar(redLowH,lowSat,lowVal),
                  cv::Scalar(redHighH,highSat,highVal), redBinFrame);
      cv::erode(redBinFrame, redBinFrame, cv::getStructuringElement(
                cv::MORPH_ELLIPSE, erodeDilateSize));
      cv::dilate(redBinFrame, redBinFrame, cv::getStructuringElement(
                 cv::MORPH_ELLIPSE, erodeDilateSize));

      // find moments of the image
  		m = cv::moments(redBinFrame,true);
  		redLocation = cv::Point(m.m10/m.m00, m.m01/m.m00);

  		// coordinates of centroid
  		// std::cout<< cv::Mat(redLocation)<< std::flush;


      cv::cvtColor(redBinFrame, redBinFrame, cv::COLOR_GRAY2BGR);
  		cv::circle( redBinFrame, redLocation,
  		15,
  		cv::Scalar(255,
  		0,0),
  		2, 8, 0 );

      ///////Decide which ball to catch and command using catchBall///////
      // Use globals redLocation, greenLocation, and blueLocation
      redBall_y = int(redLocation.y);
      greenBall_y = int(greenLocation.y);
      blueBall_y = int(blueLocation.y);

      
      if (redBall_y < leftBinPixel.y)
        catchBall("red");
      else if (greenBall_y < leftBinPixel.y)
        catchBall("green");
      else
        catchBall("blue");
      
      /* This one sucks
      if (blueBall_y > greenBall_y && blueBall_y > redBall_y) 
        catchBall("blue");
      else if (greenBall_y > redBall_y) 
        catchBall("green");
      else 
        catchBall("red");
      */

      // else if (greenBall_y ) 
      // if(redBall_y < (yDistThresh + (leftBinPixel.y+rightBinPixel.y)/2)) {
      //   catchBall("red");
      //   if ((greenBall_y > blueBall_y || catchWhich == 1) && (catchWhich != 2)) {
      //     catchBall("green");
      //     catchWhich = 1;
      //     if(greenBall_y > (yDistThresh + (leftBinPixel.y+rightBinPixel.y)/2)) {
      //       catchBall("blue");
      //     }
      //   }
      //   else {
      //       catchBall("blue");
      //       catchWhich = 2;
      //       if(blueBall_y > (yDistThresh + (leftBinPixel.y+rightBinPixel.y)/2)) {
      //         catchBall("green");
      //       }
      //   }
      // }
     


      // Show image
      cv::imshow("Camera Input", inFrame);
      cv::imshow("Blue Input", blueBinFrame);
      cv::imshow("Green Input", greenBinFrame);
      cv::imshow("Red Input", redBinFrame);

      // enter a value greater than 0 to break out of the loop
      if(cv::waitKey(10) == 32) break;




      /*
      // Command structure is very simple
      // "h\n" is to home the motor
      // "g<integer range 7 to 53>\n" sends the motor to that position in cm
      // e.g. "g35\n" sends the motor to 35cm from left wall
      if(frameCounter%200==0) {
        //sendCommand("g10\n");
      }
      else if(frameCounter%100==0){
        //sendCommand("g50\n");
      }
      */

    }
  }
  return 0;
}
