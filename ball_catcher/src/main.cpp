#include <ball_catcher/estimate_trajectory_hw.h>

int counter = 0;

int main()
{
    vector<double> xv;
    vector<double> yv;
    vector<double> zv;
    Vector3f xyz;
    TrajectoryEstimator trj;

    trj.init(false);

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

    double x_predicted = cx[2];
    double y_predicted = cy[2];
    cout << "\nx = " << x_predicted << endl;
    cout << "y = " << y_predicted << endl;
    
    return 0;
}