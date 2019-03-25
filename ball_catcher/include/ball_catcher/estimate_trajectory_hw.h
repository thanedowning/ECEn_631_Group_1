#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class TrajectoryEstimator
{
public:
    struct CamData {
        string name;
        cv::Mat mtx, dist, R, P, Q, img_0;
        cv::Rect roi;
        vector<cv::KeyPoint> keypoints;
        cv::Point2f loc;
    };
    
    TrajectoryEstimator(bool display, cv::Mat img_0L, cv::Mat img_0R);
    ~TrajectoryEstimator() {};
    void init(bool display);
    Vector3f run(cv::Mat, cv::Mat);
    vector<double> estimate();
    bool find_ball(CamData &cam, cv::Mat img);
    void find_middle(vector<cv::KeyPoint> &kps);
    void track_ball(CamData &cam, cv::Mat img);
    void display_img(CamData &cam, cv::Mat img, string title);
    const cv::SimpleBlobDetector::Params set_params();
    void create_windows();
    void setup_cam(CamData &cam, string param_file);
    Vector3f calc_3dpoints();
    const cv::SimpleBlobDetector::Params set_params();

    vector<double> xv;
    vector<double> yv;
    vector<double> zv;

private:
    CamData camL, camR;
    cv::Ptr<cv::SimpleBlobDetector> detector;
    bool display_;
};