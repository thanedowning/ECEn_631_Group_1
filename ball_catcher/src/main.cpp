#include <ball_catcher/estimate_trajectory.h>


int main()
{
    vector<double> xv;
    vector<double> yv;
    vector<double> zv;
    Vector3f xyz;
    TrajectoryEstimator trj = TrajectoryEstimator();

    trj.init(true);

    for (int i=0; i < 100; i++)
    {
        xyz = trj.run(i);
        if ( !xyz.isZero() )
        {
            xv.push_back(xyz[0]);
            yv.push_back(xyz[1]);
            zv.push_back(xyz[2]);
        }
    }
    VectorXf x(xv.size());
    VectorXf y(yv.size());
    VectorXf z(zv.size());
    
    return 0;
}