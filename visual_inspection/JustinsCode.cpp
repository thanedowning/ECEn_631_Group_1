/*
 * main.cpp
 *
 *  Created on: Jan 17, 2019
 *      Author: justin
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
using namespace cv;

static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

//square detection
// returns sequence of squares detected on the image.
static void findSquares( Mat &image, Mat &output )
{
	int thresh = 50, N = 11;
	std::vector<std::vector<Point> > squares;
    squares.clear();
    Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    std::vector<std::vector<Point> > contours;
    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }
            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
            std::vector<Point> approx;
            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(approx)) > 1000 &&
                    isContourConvex(approx) )
                {
                    double maxCosine = 0;
                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
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
	    const Point* p = &squares[i][0];
	    int n = (int)squares[i].size();
	    polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
	}
	output = image;
}


// circle detection
void circleDetect(Mat &input, Mat &output){
	Mat gray;
    cvtColor(input, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    std::vector<Vec3f> circles;

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

    HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                 gray.rows/50,  // change this value to detect circles with different distances to each other
                 150, 30, 10, 50 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        circle( output, center, 1, Scalar(0,100,100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle( output, center, radius, Scalar(255,0,255), 3, LINE_AA);
    }
}

// lines
void drawLines(Mat &input, Mat &output){

	Mat dst, cdst;
    Canny(input, dst, 50, 200, 3); 

    // void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0 )
    cvtColor(dst, cdst, CV_GRAY2BGR); 
 
    std::vector<Vec2f> lines;

    // detect lines
    HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 );
 
    // draw lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
    }

    output = cdst;
    //cvtColor(output,output,COLOR_GRAY2BGR);
}

// corner detection with subpixels
void corners2(Mat &input, Mat &output){

  

  /// Parameters for Shi-Tomasi algorithm
  std::vector<Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int maxCorners = 4;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;
  Mat grayScale;
  RNG rng(12345);

  

  /// Copy the source image
  Mat copy;
  copy = input.clone();

  cvtColor( input, grayScale, CV_BGR2GRAY );

  /// Apply corner detection
  goodFeaturesToTrack( grayScale,
                       corners,
                       maxCorners,
                       qualityLevel,
                       minDistance,
                       Mat(),
                       blockSize,
                       useHarrisDetector,
                       k );


  /// Draw corners detected
  //std::cout<<"** Number of corners detected: "<<corners.size()<<std::endl;
  int r = 6; //radius
  
  //cv2.circle(img, center, radius, color, thickness=1, lineType=8, shift=0)
  for( int i = 0; i < corners.size(); i++ ){
      	circle( copy, corners[i], 
		r, 
		Scalar(255, 
		255,255), 
		-1, 8, 0 ); 
  }

  output = copy;
  //cvtColor(output,output,COLOR_GRAY2BGR);
}

// harris corners
void corners(Mat &input, Mat &output){

	Mat grayScale;
	int thresh = 150; //max 255
	int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    // create a new matrix
	Mat dst = Mat::zeros( input.size(), CV_32FC1 );

    cvtColor( input, grayScale, CV_BGR2GRAY );
   
    cornerHarris(grayScale, dst, blockSize, apertureSize, k );
    
    
    Mat dst_norm, dst_norm_scaled;
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );

    
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresh )
            {
                circle( dst_norm_scaled, Point(j,i), 5,  Scalar(0), 2, 8, 0 );
            }
        }
    }

	output = dst_norm_scaled;
	cvtColor(output,output,COLOR_GRAY2BGR);
}

// Threshold
void thresholder(Mat &input,Mat &output){
	// Convert the image to Gray
	Mat grayScale;
	cvtColor( input, grayScale, CV_BGR2GRAY );

	/*
		 0: Binary
		 1: Binary Inverted
		 2: Threshold Truncated
		 3: Threshold to Zero
		 4: Threshold to Zero Inverted
	*/
	threshold(grayScale, output, 100, 255,0);
	cvtColor(output,output,COLOR_GRAY2BGR);
}

void difference(Mat prevInput,Mat currentInput, Mat &output){
	Mat grayScale1, grayScale2;

	//cvtColor( currentInput, grayScale1, CV_BGR2GRAY );
	//cvtColor( prevInput, grayScale2, CV_BGR2GRAY );

	absdiff(prevInput, currentInput, output);
	thresholder(output, output);
	//erode(output, output, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
}

// Canny Edges
void canny(Mat &input, Mat &output){
	Mat dst;
	Mat grayScale;
	Mat detected_edges;

	int ratio = 3;
	int kernel_size = 3;
	int lowThreshold = 75;

	// Create a matrix of the same type and size as src (for dst)
 	dst.create( input.size(), input.type() );

  	// Convert the image to grayscale
  	cvtColor( input, grayScale, CV_BGR2GRAY );

	/// Reduce noise with a kernel 3x3
	blur( grayScale, detected_edges, Size(3,3) );

	/// Canny detector
	Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);

	input.copyTo( dst, detected_edges);
	output = detected_edges;
	cvtColor(output,output,COLOR_GRAY2BGR);
	//output = dst // Use to maintain color of the lines;
}


int main( int argc, char** argv )
{
	char keyPress;
	char TempKey_press;
	VideoCapture video(0); // get a camera object
	Mat frame; // allocate an image buffer object
	Mat outFrame;
	Mat prevFrame;
	
 	// initialize a display window
	namedWindow("Roberts", CV_WINDOW_AUTOSIZE);

	// Default resolution of the frame is obtained.The default resolution is system dependent.
	int frame_width = video.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = video.get(CV_CAP_PROP_FRAME_HEIGHT);

	// Create a video write object.
	VideoWriter VOut;


	// Initialize video write object (only done once). Change frame size to match your camera resolution.
	VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
	//VOut.open("VideoOut.avi", -1 , 30, Size(frame_width, frame_height), 1); // use this if you don’t have the correct codec

	//initialize frames
	video >> prevFrame;
	video >> frame;
	outFrame = frame;



	for(int i = 0; i < 5000; i++){
		

		// wait for image processing key
		TempKey_press = (char)waitKey(1);

		// if there was a change, change the keyPress option
		if(TempKey_press == 'w' || TempKey_press == 'q' || TempKey_press == 't' || TempKey_press == 'e' || TempKey_press == 'c' || TempKey_press == 'l' || TempKey_press == 'd' || TempKey_press == 'n'){
			keyPress = TempKey_press;
		}

		//different image processing options 
		if(keyPress == 't'){
			thresholder(frame,outFrame);
		}else if(keyPress == 'e'){
			canny(frame,outFrame);
		}else if(keyPress == 'c'){
			corners2(frame,outFrame);
		}else if(keyPress == 'l'){
			drawLines(frame,outFrame);
		}else if(keyPress == 'd'){
			difference(prevFrame,frame,outFrame);
		}else if(keyPress == 'q'){
			circleDetect(frame, outFrame);
		}else if(keyPress == 'w'){
			findSquares(frame,outFrame);
		}else if(keyPress == 'n'){
			outFrame = frame;
		}
		
		
		

		imshow("Roberts", outFrame);
		waitKey(0);
		//VOut << outFrame;

		// get another frame from video
		frame.copyTo(prevFrame);
		video >> frame;

	}

	// When everything done, release the video capture and write object
	video.release();
	VOut.release();

	// Closes all the windows
	destroyAllWindows();

	return 0;
}




