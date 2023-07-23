#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class basic_spline
{
public:
    vector<double> x,y;
    vector<double> a, b, c, d;

    basic_spline(vector<double> input_x, vector<double> input_y);

    double calculate_position(double val);

    double calculate_first_d(double val);

    double calculate_second_d(double val);

    MatrixXf calculate_A(vector<double> h);

    VectorXf calculate_B(vector<double> h);

    double calculate_curvature(double val);

    int binary_search(vector<double> list, double val);

};

class spline2D
{
public:
    basic_spline* x;
    basic_spline* y;
    vector<double> s;

    spline2D(vector<double> input_x, vector<double> input_y);

    double* calculate_position(double val);

    double calculate_curvature(double val); 

    double calculate_yaw(double val);
};

class spline2D_interpolated
{
public:
    vector<double> r_x, r_y, r_yaw, r_k, travel;

    spline2D_interpolated(vector<double> input_x, vector<double> input_y, double ds);

    double* calc_nearest_index(double cur_x, double cur_y, int index);

    double pi_2_pi(double angle);
};

class spline3D
{
public:
    basic_spline* x;
    basic_spline* y;
    basic_spline* z;
    vector<double> s;

    spline3D(vector<double> input_x, vector<double> input_y, vector<double> input_z);

    double* calculate_position(double val);

    double calculate_curvature(double val); 

    double calculate_curve_p(double val);

    double calculate_yaw(double val);
    
    double calculate_pitch(double val);
};

class spline3D_interpolated
{
public:
    vector<double> r_x, r_y, r_z, r_yaw, r_k, travel, r_pitch, r_pitch_p;

    spline3D_interpolated(vector<double> input_x, vector<double> input_y, vector<double> input_z, double ds);

    double* calc_nearest_index(double cur_x, double cur_y, double cur_z, double cur_yaw, int index);

    double pi_2_pi(double angle);
};