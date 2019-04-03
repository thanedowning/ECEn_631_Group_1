#include "stdafx.h"
#include "estimate_trajectory_hw.h"

using namespace std;

TrajectoryEstimator::TrajectoryEstimator(bool display, cv::Mat img_0L, cv::Mat img_0R)
{   
    display_ = display;
    camL.name = "Left";
    camR.name = "Right";

	cv::imshow("Justin is Cool", img_0L);
	cv::waitKey(0);

	cv::cvtColor(img_0L, img_0L, cv::COLOR_BGR2GRAY);
	cv::cvtColor(img_0R, img_0R, cv::COLOR_BGR2GRAY);

    img_0L.copyTo(camL.img_0);
    img_0R.copyTo(camR.img_0);

    int w(200);
    camL.roi = cv::Rect(350, 0, w, w);
    camR.roi = cv::Rect(100, 0, w, w);

    const cv::SimpleBlobDetector::Params params = set_params();
    detector = cv::SimpleBlobDetector::create(params);

    // Get intrinsics
    setup_cam(camL, "left_cam.yaml");
    setup_cam(camR, "right_cam.yaml");
    // Get extrinsic parameters
    cv::FileStorage fin("stereo.yaml", cv::FileStorage::READ);
    cv::Mat R, T, E, F;
    fin["R"] >> R;
    fin["E"] >> E;
    fin["T"] >> T;
    fin["F"] >> F;
    fin.release();
    
    create_windows();

    // Rectification params
    cv::Mat Q;
    cv::stereoRectify(camL.mtx, camL.dist, camR.mtx, camR.dist, cv::Size(w,w), 
                      R, T, camL.R, camR.R, camL.P, camR.P, Q);
    camL.Q = Q;
    camR.Q = Q;
}

Vector3f TrajectoryEstimator::run(cv::Mat imgL, cv::Mat imgR)    
{   
    // int t = 0;
    Vector3f pt3d;
    bool ballL_found = find_ball(camL, imgL);
    bool ballR_found = find_ball(camR, imgR);
    if (ballL_found && ballR_found) 
    {   
        // t = 100;
        track_ball(camL, imgL);
        track_ball(camR, imgR);
        pt3d = calc_3dpoints();

        xv.push_back(pt3d[0]);
        yv.push_back(pt3d[1]);
        zv.push_back(pt3d[2]);
    }
    else
    {
        // t = 10;
        // display_img(camL, camL.imgs[i], camL.name);
        // display_img(camR, camR.imgs[i], camR.name);
        pt3d = Vector3f::Zero();
    }
    // if (cv::waitKey(t) == 113) exit(0);

    return pt3d;
}

vector<double> TrajectoryEstimator::estimate()
{
    VectorXf x(xv.size());
    VectorXf y(yv.size());
    VectorXf z(zv.size());
    MatrixXf A(xv.size(), 3);
    Vector3f a;
    VectorXf bx(xv.size());
    VectorXf by(xv.size());
    Vector3f cx;
    Vector3f cy;
    
    for (int i=0; i < xv.size(); i++)
    {
        x[i] = xv[i];
        y[i] = yv[i];
        z[i] = zv[i];

        a << z[i]*z[i], z[i], 1;
        A.row(i) = a;

        bx[i] = x[i]; 
        by[i] = y[i]; 
    }

    cx = A.colPivHouseholderQr().solve(bx);
    cy = A.colPivHouseholderQr().solve(by);

    cout << "size: " << bx.size() << endl;
    cout << "cx: " << cx << endl;
    cout << "cy: " << cy << endl;

    vector<double> prediction{cx[2], cy[2]};
    return prediction;
}

void TrajectoryEstimator::setup_cam(CamData &cam, string param_file)
{
    cv::FileStorage fin(param_file, cv::FileStorage::READ);
    fin["mtx"] >> cam.mtx;
    fin["dist"] >> cam.dist;
    fin.release();
}

bool TrajectoryEstimator::find_ball(CamData &cam, cv::Mat img)
{
    bool found_ball(false);

    vector<cv::KeyPoint> kps;

    img = img(cam.roi);
	cv::cvtColor(img, img, CV_BGR2GRAY);

    cv::Mat bin;
    cv::absdiff(cam.img_0, img, bin);
    cv::threshold(bin, bin, 20, 255, 0);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));
    cv::erode(bin, bin, element);
    cv::dilate(bin, bin, element);

    detector->detect(bin, kps);

    if (kps.size()) 
    { 
        found_ball = true; 
        if (kps.size() > 1) { find_middle(kps); }
        cv::Point2f pt(kps[0].pt.x, kps[0].pt.y);
        pt.x += cam.roi.x;
        pt.y += cam.roi.y;
        cam.loc = pt;
    }

    // string title = "Binary " + cam.name;
    // display_img(cam, bin, title);

    return found_ball;
}

void TrajectoryEstimator::find_middle(vector<cv::KeyPoint> &kps)
{   
    cv::KeyPoint kp(0,0,0);
    for (int i=0; i < kps.size(); i++)
    {
        kp.pt.x += kps[i].pt.x;
        kp.pt.y += kps[i].pt.y;
        if (kps[i].size > kp.size)
            kp.size = kps[i].size;
    }

    kp.pt.x /= kps.size();
    kp.pt.y /= kps.size();

    vector<cv::KeyPoint> new_vec;
    new_vec.push_back(kp);
    kps = new_vec;
}

void TrajectoryEstimator::track_ball(CamData &cam, cv::Mat img)
{
    // Undistort
    vector<cv::Point2f> pts{cam.loc};
    cv::undistortPoints(pts, pts, cam.mtx, cam.dist, cam.R, cam.P);
    cam.loc = pts[0];
    
    cv::Mat img_kps;
    cam.keypoints[0].pt = cam.loc;
    // cv::drawKeypoints(img, cam.keypoints[i], img_kps, 
        // cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    
    // cv::Point2f center;
    // center.x = cam.loc.x-cam.roi.x;
    // center.y = cam.loc.y-cam.roi.y;
    // cv::circle(img_kps, center, 10, cv::Scalar(255,0,0));

    // display_img(cam, img_kps, cam.name);
}

Vector3f TrajectoryEstimator::calc_3dpoints()
{
    vector<cv::Point3f> persp_pts, pts_3d;
    cv::Point3f pt(camL.loc.x, camL.loc.y, camL.loc.x - camR.loc.x);
    persp_pts.push_back(pt);

    // Get 3d ball location in left camera frame 
    cv::perspectiveTransform(persp_pts, pts_3d, camL.Q);

    // OFFSETS //
    pts_3d[0].x -= 10.135;
    pts_3d[0].y -= 29.0;
    pts_3d[0].z -= 21.0;

    Vector3f pt3;
    pt3[0] = pts_3d[0].x;
    pt3[1] = pts_3d[0].y;
    pt3[2] = pts_3d[0].z;
    return pt3;
}

const cv::SimpleBlobDetector::Params TrajectoryEstimator::set_params()
{
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 100;
    params.maxThreshold = 255;
    params.filterByColor = true;
    params.blobColor = 255;
 
    return params;
}

void TrajectoryEstimator::display_img(CamData &cam, cv::Mat img, string title)
{   
    if (!display_) { return; }

    cv::Mat enlarged;
    cv::resize(img, enlarged, cv::Size(img.cols*2.5, img.rows*2.5), cv::INTER_NEAREST);
    cv::imshow(title, enlarged);
}

void TrajectoryEstimator::create_windows()
{   
    if (!display_) { return; }
    cv::namedWindow(camL.name);
    cv::namedWindow(camR.name);
    string binL = "Binary " + camL.name;
    string binR = "Binary " + camR.name;
    cv::namedWindow(binL);
    cv::namedWindow(binR);

    int x1(100), x2(610), y2(575);
    cv::moveWindow(camL.name, x1, 0);
    cv::moveWindow(camR.name, x2, 0);
    cv::moveWindow(binL, x1,y2);
    cv::moveWindow(binR, x2,y2);
}