#include <iostream>
#include <ctype.h>
#include <string>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

static void intro_message() {
  std::cout << "Project 1 for ECEn 631: a Chex Mix Classifier.\n";

}

static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

//square detection
// returns sequence of squares detected on the image.
int findSquares( cv::Mat &image, cv::Mat &output )
{
	int thresh = 50, N = 11, upper_limit_area = 10000, lower_limit_area = 1000;
	std::vector<std::vector<cv::Point> > squares;
    squares.clear();
    cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    // down-scale and upscale the image to filter out the noise
    cv::pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
    cv::pyrUp(pyr, timg, image.size());
    std::vector<std::vector<cv::Point> > contours;
    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);
        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cv::Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                cv::dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }
            // find contours and store them all as a list
            cv::findContours(gray, contours, cv::RETR_LIST,
                             cv::CHAIN_APPROX_SIMPLE);
            std::vector<cv::Point> approx;
            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                cv::approxPolyDP(contours[i], approx,
                                 cv::arcLength(contours[i], true)*0.02, true);
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(cv::contourArea(approx)) > lower_limit_area &&
                    cv::isContourConvex(approx) &&
                    fabs(cv::contourArea(approx)) < upper_limit_area)
                {
                    double maxCosine = 0;
                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2],
                                                 approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }


	for( size_t i = 0; i < squares.size(); i++ )
	{
	    const cv::Point* p = &squares[i][0];
	    int n = (int)squares[i].size();
	    //cv::polylines(image, &p, &n, 1, true, cv::Scalar(0,255,0), 3, cv::LINE_AA);
	}
	output = image;
  if (squares.size() > 0) {
    return 1;
  }
  else {
    return 0;
  }
}


// circle detection
int findCircles(cv::Mat &input, cv::Mat &output){
  cv::Mat gray;
  cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);

  // erode out the noise
	cv::erode(gray, gray,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));

	// dilate again to fill in holes
	cv::dilate(gray, gray,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));
  //cv::medianBlur(gray, gray, 5);
  std::vector<cv::Vec3f> circles;
  int inner_radius = 25, outer_radius = 150, accumulation_thresh = 14;

	/*  Hough Circle parameters
	src_gray: Input image (grayscale)
	circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
	CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
	dp = 1: The inverse ratio of resolution
	min_dist = src_gray.rows/8: Minimum distance between detected centers
	param_1 = 200: Upper threshold for the internal Canny edge detector
	param_2 = 100*: Threshold for center detection.
	min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
	max_radius = 0: Maximum radius to be detected. If unknown, put zero as default
	*/

  cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
               gray.rows/3,  // change this value to detect circles with different distances to each other
               150, accumulation_thresh, inner_radius, outer_radius // change the last two parameters
          // (min_radius & max_radius) to detect larger circles
  );
  for( size_t i = 0; i < circles.size(); i++ )
  {
    cv::Vec3i c = circles[i];
    cv::Point center = cv::Point(c[0], c[1]);
    // circle center
    //cv::circle( output, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
    // circle outline
    int radius = c[2];
    //cv::circle( output, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
  }
  if (circles.size() > 0) {
    return 1;
  }
  else {
    return 0;
  }
}

int findRye(cv::Mat &input, cv::Mat &mask, cv::Mat &output) {
  int rye_thresh = 78, belt_thresh = 55, max_BINARY_value = 255;
  int belt_bright_pixels = 0, belt_pixel_tresh = 8000;
  int rye_bright_pixels = 0, rye_pixel_tresh = 6000;
  cv::Mat grayFrame, tmp;
  cv::cvtColor(input, grayFrame, cv::COLOR_BGR2GRAY);
  cv::threshold(grayFrame, tmp, belt_thresh, max_BINARY_value, cv::THRESH_BINARY);
  belt_bright_pixels = countNonZero(tmp);
  if (belt_bright_pixels > belt_pixel_tresh) {
    cv::erode(grayFrame, grayFrame,
              cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    cv::dilate(grayFrame, grayFrame,
               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    cv::threshold(grayFrame, tmp, rye_thresh, max_BINARY_value, cv::THRESH_BINARY);
    rye_bright_pixels = countNonZero(tmp);
    if (rye_bright_pixels < rye_pixel_tresh) {
      return 1;
    }
  }
  return 0;
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
  bool show_shapes = false;
  bool show_circles = false;
  bool show_squares = false;

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

  cv::Mat inFrame, colorFrame, grayFrame, outFrame, tmpFrame, prevFrame, mask;

  mask = imread("belt_mask.jpg", cv::IMREAD_GRAYSCALE);

  std::vector<cv::Point2f> corners;
  std::vector<cv::Vec4i> lines;
  std::vector<cv::Vec3f> circles;

  int binary_threshold_value = 80;
  int max_BINARY_value = 255;
  double canny_threshold_1 = 15;
  double canny_threshold_2 = 80;
  int stay_count = 0;
  int square_found_count = 0;
  int rye_found_count = 0;
  int circle_found_count = 0;

  while (true) {
    vid >> inFrame;

    if (inFrame.empty())
      break;

    cv::flip(inFrame, colorFrame, 1);
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_BGR2GRAY);

    if (show_normal){
      cv::swap(colorFrame, outFrame);
    }

    if (show_bin) {
      //cv::erode(grayFrame, grayFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    	//cv::dilate(grayFrame, grayFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
      cv::threshold(grayFrame, grayFrame, binary_threshold_value, max_BINARY_value,
                    cv::THRESH_BINARY);
      cv::threshold(mask, mask, binary_threshold_value, max_BINARY_value,
                    cv::THRESH_BINARY);
      bitwise_and(grayFrame, mask, outFrame);
      std::cout << countNonZero(outFrame) << std::endl;
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

    if (show_squares) {
      std::cout << findSquares(colorFrame, outFrame) << " Squares" << std::endl;
    }

    if (show_circles) {
      std::cout <<  findCircles(colorFrame, outFrame) << " Circles" << std::endl;
    }

    if (show_shapes) {
      stay_count++;

      if (findSquares(colorFrame, outFrame)) {
        square_found_count++;
        rye_found_count = 0;
        stay_count = 0;
        if (square_found_count > 2)
        std::cout << "Chex: Good" << std::endl;
      }
      else if (findCircles(colorFrame, outFrame)) {
        circle_found_count++;
        stay_count = 0;
        if (circle_found_count > 2)
        std::cout << "Pret: Bad" << std::endl;
      }
      else if (findRye(colorFrame, mask, outFrame) && circle_found_count < 2) {
        rye_found_count++;
        stay_count = 0;
        if (rye_found_count > 7)
          std::cout << "Rye: Ugly" << std::endl;
      }
      if (stay_count != 0 && (stay_count % 10) == 0) {
        std::cout << std::endl;
        square_found_count = 0;
        rye_found_count = 0;
        circle_found_count = 0;
      }
    }

    //vidout.write(outFrame);
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

      show_shapes = false;
      show_circles = false;
      show_squares = false;
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
      show_shapes = true;
    }
    if (input == 's'){
      show_squares = true;
      show_circles = false;
    }
    if (input == 'c'){
      show_circles = true;
      show_squares = false;
    }
  }
  return 0;
}
