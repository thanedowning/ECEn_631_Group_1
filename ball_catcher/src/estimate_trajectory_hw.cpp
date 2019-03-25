#include <ball_catcher/estimate_trajectory_hw.h>

using namespace std;

void TrajectoryEstimator::init(bool display)
{   
    display_ = display;
    camL.name = "Left";
    camR.name = "Right";

    int w(200);
    camL.roi = cv::Rect(350, 0, w, w);
    camR.roi = cv::Rect(100, 0, w, w);

    const cv::SimpleBlobDetector::Params params = set_params();
    detector = cv::SimpleBlobDetector::create(params);

    // Get intrinsics
    setup_cam(camL, "params/left_cam.yaml");
    setup_cam(camR, "params/right_cam.yaml");
    // Get extrinsic parameters
    cv::FileStorage fin("params/stereo.yaml", cv::FileStorage::READ);
    cv::Mat R, T, E, F;
    fin["R"] >> R;
    fin["E"] >> E;
    fin["T"] >> T;
    fin["F"] >> F;
    fin.release();

    // Rectification params
    cv::Mat Q;
    cv::stereoRectify(camL.mtx, camL.dist, camR.mtx, camR.dist, cv::Size(w,w), 
                      R, T, camL.R, camR.R, camL.P, camR.P, Q);
    camL.Q = Q;
    camR.Q = Q;
}

Vector3f TrajectoryEstimator::run(int i)    
{   
    int t = 0;
    Vector3f pt3;
    bool ballL = find_ball(camL, i);
    bool ballR = find_ball(camR, i);
    if (ballL && ballR) 
    {   
        t = 100;
        track_ball(camL, i);
        track_ball(camR, i);
        pt3 = calc_3dpoints(i);
    }
    else
    {
        t = 10;
        display_img(camL, camL.imgs[i], camL.name);
        display_img(camR, camR.imgs[i], camR.name);
        pt3 = Vector3f::Zero();
    }
    if (cv::waitKey(t) == 113) exit(0);

    return pt3;
}

void TrajectoryEstimator::setup_cam(CamData &cam, string param_file)
{
    cv::FileStorage fin(param_file, cv::FileStorage::READ);
    fin["mtx"] >> cam.mtx;
    fin["dist"] >> cam.dist;
    fin.release();
}

bool TrajectoryEstimator::find_ball(CamData &cam, int i)
{
    bool found_ball(false);

    vector<cv::KeyPoint> kps;
    vector<cv::Point2f> pts;

    cv::Mat img, gray, enlarged;
    img = cv::imread(string(cam.img_files[i]));
    img = img(cam.roi);
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cam.imgs.push_back(gray);
    if (i == 0)
    {
        cam.keypoints.push_back(kps);
        cam.points.push_back(pts);
        return false;
    }

    cv::Mat bin;
    cv::absdiff(cam.imgs[0], gray, bin);
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
        pts.push_back(pt);
    }

    cam.keypoints.push_back(kps);
    cam.points.push_back(pts);
    
    string title = "Binary " + cam.name;
    display_img(cam, bin, title);

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

void TrajectoryEstimator::track_ball(CamData &cam, int i)
{
    // Undistort
    cv::undistortPoints(cam.points[i], cam.points[i], 
                             cam.mtx, cam.dist, cam.R, cam.P);
    
    cv::Mat img_kps;
    cam.keypoints[i][0].pt = cam.points[i][0];
    cv::drawKeypoints(cam.imgs[i], cam.keypoints[i], img_kps, 
        cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    
    cv::Point2f center;
    center.x = cam.points[i][0].x-cam.roi.x;
    center.y = cam.points[i][0].y-cam.roi.y;
    cv::circle(img_kps, center, 10, cv::Scalar(255,0,0));

    display_img(cam, img_kps, cam.name);
}

Vector3f TrajectoryEstimator::calc_3dpoints(int i)
{
    vector<cv::Point3f> persp_pts, pts_3d;
    cv::Point3f pt(camL.points[i][0].x, camL.points[i][0].y, 
                    camL.points[i][0].x - camR.points[i][0].x);
    persp_pts.push_back(pt);

    // Get 3d ball location in left camera frame 
    cv::perspectiveTransform(persp_pts, pts_3d, camL.Q);
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