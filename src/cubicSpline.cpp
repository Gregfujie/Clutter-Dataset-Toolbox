#include "cubicSpline.h"
#include <iostream>
#include <fstream>

basic_spline::basic_spline(vector<double> input_x, vector<double> input_y){
    this->x = input_x;
    this->y = input_y;
    vector<double> h;
    int length = this->x.size();
    // cout<<"h"<<endl;
    for(int i = 0; i<length-1; i++){
        h.emplace_back(this->x[i+1] - this->x[i]);
        // cout<<h[i] <<" ";
    }
    // cout << endl;

    for(int i = 0; i<length; i++){
        this->a.emplace_back(this->y[i]);
    }

    MatrixXf A = calculate_A(h);
    // cout<<"A"<<endl;
    // for(int i = 0; i<length; i++){
    //     for(int j = 0; j<length; j++){
    //         cout<<A(i, j)<<" ";
    //     }
    //     cout << endl;
    // }

    VectorXf B = calculate_B(h);
    // cout<<"B"<<endl;
    // for(int i = 0; i<length; i++){
    //     cout<<B[i]<<" ";
    // }
    // cout<<endl;

    VectorXf solution = A.lu().solve(B); 

    for(int i = 0; i<solution.size(); i++){
        this->c.emplace_back(solution[i]);
    }
    // for(int i = 0; i<this->c.size();i++){
    //     cout<<this->c[i]<<" ";
    // }
    // cout<<endl;

    for(int i = 0; i<length-1; i++){
        this->d.emplace_back((this->c[i+1] - this->c[i])/(3.0 * h[i]));
        double cur_b = (this->a[i+1] - this->a[i]) / h[i] 
        - h[i] * (this->c[i+1] + 2.0 * this->c[i]) / 3.0;
        this->b.emplace_back(cur_b);
    }
    // cout<<"b"<<endl;
    // for(int i = 0; i<this->b.size();i++){
    //     cout << b[i] <<" ";
    // }
    // cout<<endl;
    // cout<<"d"<<endl;
    // for(int i = 0; i<this->d.size();i++){
    //     cout<< d[i] << " ";
    // }
    // cout<<endl;

    cout << "basic_complete"<<endl;

}

double basic_spline::calculate_position(double val){
    if (val < this->x[0]){
        return NULL;
    }
    else if (val > this->x[this->x.size()-1]){
        return NULL;
    }
    int index = binary_search(this->x, val);
    double dx = val - this->x[index];
    double result = this->a[index] + this->b[index] * dx + this->c[index] * pow(dx, 2.0) + this->d[index] *pow(dx, 3.0);
    return result;
}

double basic_spline::calculate_first_d(double val){
    if (val < this->x[0]){
        return NULL;
    }
    else if (val > this->x[this->x.size()-1]){
        return NULL;
    }
    int index = binary_search(this->x, val);
    double dx = val - this->x[index];
    double result = this->b[index]+2.0 * this->c[index]*dx + 3.0 * this->d[index] *pow(dx, 2.0);
    return result;
}

double basic_spline::calculate_second_d(double val){
    if (val < this->x[0]){
        return NULL;
    }
    else if (val > this->x[this->x.size()-1]){
        return NULL;
    }
    int index = binary_search(this->x, val);
    double dx = val - this->x[index];
    double result = 2.0 * this->c[index] + 6.0 * this->d[index] *dx;
    return result;
}

MatrixXf basic_spline::calculate_A(vector<double> h){
    int length = h.size()+1;
    MatrixXf A = MatrixXf::Zero(length,length);
    A(0, 0) = 1.0;
    for(int i = 0; i<length - 1; i++){
        if (i != length - 2){
            A(i+1, i+1) = 2.0*(h[i] + h[i+1]);
        }
        A(i+1, i) = h[i];
        A(i, i+1) = h[i];
    }
    A(0, 1) = 0.0;
    A(length-1, length - 2) = 0.0;
    A(length-1, length - 1) = 1.0;
    return A;
}

VectorXf basic_spline::calculate_B(vector<double> h){
    int length = h.size()+1;
    VectorXf B = VectorXf::Zero(length);
    for(int i = 0; i<length-2; i++){
        B[i+1] = 3.0 * (this->a[i+2] - this->a[i+1]) / h[i+1]
        - 3.0*(this->a[i+1] - this->a[i]) / h[i];
    }
    return B;
}

double basic_spline::calculate_curvature(double val){
    int index = int(floor(val));
    if(index < 0){index = 0;}
    else if(index >= this->a.size()){ index = this->a.size()-1;}
    double dt = val - double(index);
    double df = this->b[index] + 2.0 * this->c[index] *dt + 3.0*this->d[index] *dt*dt;
    double ddf = 2.0 *this->c[index] + 6.0*this->d[index] *dt;
    double k = ddf/(pow((1+df*df), 1.5));
    return k;
}

int basic_spline::binary_search(vector<double> list, double val){
    int left = 0;
    int right = list.size() - 1;
    while(left<=right){
        int mid = left + (right - left) / 2;
        if(mid >= 1 && list[mid] > val && list[mid - 1]<=val){return mid - 1;}
        else if(mid < 1 && list[mid]<=val){return mid;}
        else if(list[mid]<=val){left = mid + 1;}
        else{right = mid - 1;}
    }
    if(left == 0){return 0;}
    else{ return list.size() - 1;}
}


spline2D::spline2D(vector<double> input_x, vector<double> input_y){
    vector<double> dx;
    vector<double> dy;
    int length = input_x.size();
    for(int i = 0; i<length-1; i++){
        dx.emplace_back(input_x[i+1]-input_x[i]);
        dy.emplace_back(input_y[i+1]-input_y[i]);
    }
    vector<double> ds;
    for(int i = 0; i<length-1;i++){
        ds.emplace_back(sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
    }
    vector<double> s;
    s.emplace_back(0.0);
    for(int i = 0; i<length-1; i++){
        s.emplace_back(s[i] + ds[i]);
    }
    this->s = s;
    this->x = new basic_spline(this->s, input_x);
    this->y = new basic_spline(this->s, input_y);
    cout<<"spline2D_complete"<<endl;
}

double* spline2D::calculate_position(double val){
    double x = this->x->calculate_position(val);
    double y = this->y->calculate_position(val);
    double* p = new double(2);
    p[0] = x;
    p[1] = y;
    return p;
}

double spline2D::calculate_curvature(double val){
    double dx = this->x->calculate_first_d(val);
    double ddx = this->x->calculate_second_d(val);
    double dy = this->y->calculate_first_d(val);
    double ddy = this->y->calculate_second_d(val);
    double k = (ddy *dx - ddx * dy) / pow((pow(dx, 2) + pow(dy, 2)), 1.5);
    return k;
}

double spline2D::calculate_yaw(double val){
    double dx = this->x->calculate_first_d(val);
    double dy = this->y->calculate_first_d(val);
    double yaw = atan2(dy, dx);
    return yaw;
}

spline2D_interpolated::spline2D_interpolated(vector<double> input_x, vector<double> input_y, double ds){
    spline2D sp = spline2D(input_x, input_y);
    vector<double> s;
    double max_val = sp.s[sp.s.size()-1];
    double val = 0.0;
    while(val+ds<max_val){
        s.emplace_back(val);
        val += ds;
    }
    for(int i = 0; i<s.size(); i++){
        double* temp = sp.calculate_position(s[i]);
        this->r_x.emplace_back(temp[0]);
        this->r_y.emplace_back(temp[1]);
        this->r_yaw.emplace_back(sp.calculate_yaw(s[i]));
        this->r_k.emplace_back(sp.calculate_curvature(s[i]));
    }
    this->travel.emplace_back(0.0);
    for(int i = 0; i<s.size()-1;i++){
        double dx = this->r_x[i+1] - this->r_x[i];
        double dy = this->r_y[i+1] - this->r_y[i];
        double cur = sqrt(dx*dx + dy*dy);
        this->travel.emplace_back(this->travel[i] + cur);
    }
    cout<<"spline2D_interpolated_complete"<<endl;
}

spline3D::spline3D(vector<double> input_x, vector<double> input_y, vector<double> input_z){
    vector<double> dx;
    vector<double> dy;
    int length = input_x.size();
    for(int i = 0; i<length-1; i++){
        dx.emplace_back(input_x[i+1]-input_x[i]);
        dy.emplace_back(input_y[i+1]-input_y[i]);
    }
    vector<double> ds;
    for(int i = 0; i<length-1;i++){
        ds.emplace_back(sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
    }
    vector<double> s;
    s.emplace_back(0.0);
    for(int i = 0; i<length-1; i++){
        s.emplace_back(s[i] + ds[i]);
    }
    this->s = s;
    this->x = new basic_spline(this->s, input_x);
    this->y = new basic_spline(this->s, input_y);
    this->z = new basic_spline(this->s, input_z);
    cout<<"spline3D_complete"<<endl;
}

double* spline3D::calculate_position(double val){
    double x = this->x->calculate_position(val);
    double y = this->y->calculate_position(val);
    double z = this->z->calculate_position(val);
    double* p = new double(3);
    p[0] = x;
    p[1] = y;
    p[2] = z;
    return p;
}

double spline3D::calculate_curvature(double val){
    double dx = this->x->calculate_first_d(val);
    double ddx = this->x->calculate_second_d(val);
    double dy = this->y->calculate_first_d(val);
    double ddy = this->y->calculate_second_d(val);
    double k = (ddy *dx - ddx * dy) / pow((pow(dx, 2) + pow(dy, 2)), 1.5);
    return k;
}

double spline3D::calculate_curve_p(double val){
    double dz = this->z->calculate_first_d(val);
    double ddz = this->z->calculate_second_d(val);
    double k = ddz / (pow((1 + pow(dz, 2)), 1.5));
    return k;
}

double spline3D::calculate_yaw(double val){
    double dx = this->x->calculate_first_d(val);
    double dy = this->y->calculate_first_d(val);
    double yaw = atan2(dy, dx);
    return yaw;
}

double spline3D::calculate_pitch(double val){
    double dx = this->z->calculate_first_d(val);
    double pitch = -atan2(dx, 1);
    return pitch;
}

spline3D_interpolated::spline3D_interpolated(vector<double> input_x, vector<double> input_y, vector<double> input_z, double ds){
    spline3D sp = spline3D(input_x, input_y, input_z);
    vector<double> s;
    double max_val = sp.s[sp.s.size()-1];
    double val = 0.0;
    while(val+ds<max_val){
        s.emplace_back(val);
        val += ds;
    }
    for(int i = 0; i<s.size(); i++){
        double* temp = sp.calculate_position(s[i]);
        this->r_x.emplace_back(temp[0]);
        this->r_y.emplace_back(temp[1]);
        this->r_z.emplace_back(temp[2]);
        this->r_yaw.emplace_back(sp.calculate_yaw(s[i]));
        this->r_k.emplace_back(sp.calculate_curvature(s[i]));
        this->r_pitch.emplace_back(sp.calculate_pitch(s[i]));
        this->r_pitch_p.emplace_back(sp.calculate_curve_p(s[i]));
    }
    this->travel.emplace_back(0.0);
    for(int i = 0; i<s.size()-1;i++){
        double dx = this->r_x[i+1] - this->r_x[i];
        double dy = this->r_y[i+1] - this->r_y[i];
        double cur = sqrt(dx*dx + dy*dy);
        this->travel.emplace_back(this->travel[i] + cur);
    }
    cout<<"spline3D_interpolated_complete"<<endl;
}

double* spline3D_interpolated::calc_nearest_index(double cur_x, double cur_y, double cur_z, double cur_yaw, int index){
    double* id = new double(3);
    double min_d;
    double min_dxy;
    double min_index;
    double min_dz;
    // int end = min(index + this->r_x.size() / 10, this->r_x.size());
    int end = min(index + 10, int(this->r_x.size()));
    // cout<<end<<endl;
    for (int i = index; i<end; i++){
        double dx = cur_x - this->r_x[i];
        double dy = cur_y - this->r_y[i];
        double dz = cur_z - this->r_z[i];
        double temp_d = abs(sqrt(dx*dx+dy*dy+dz*dz));
        double temp_dxy = abs(sqrt(dx*dx+dy*dy));
        if (i == index){
            min_d = temp_d;
            min_index = i;
            min_dxy = temp_dxy;
            min_dz = dz;
        }
        else if(temp_d < min_d){
            min_d = temp_d;
            min_index = i;
            min_dxy = temp_dxy;
            min_dz = dz;
        }
    }
    id[0] = min_index;
    id[1] = sqrt(min_dxy);
    if(id[1] < 0.01){id[1] = 0;}
    id[2] = abs(min_dz);
    double dxl = this->r_x[min_index] - cur_x;
    double dyl = this->r_y[min_index] - cur_y;
    double dzl = this->r_z[min_index] - cur_z;

    double angle = pi_2_pi(this->r_yaw[min_index] - atan2(dyl, dxl));
    // double angle = pi_2_pi(atan2(dyl, dxl) - cur_yaw);
    // if (this->r_k[id[0]] > 3.0){

    // }
    if (angle<0){
        id[1] *= -1;
    }
    if (pi_2_pi(this->r_pitch[min_index] - atan2(dzl,1)) > 0){
        id[2] *= -1;
    }
    // cout<<cur_x<<" "<<cur_y<<" "<<index<<" "<<min_index<<endl;
    return id;
}

double spline3D_interpolated::pi_2_pi(double angle){
    double temp = angle + M_PI;
    if (temp >= 0){
        while (temp > 2*M_PI){
            temp -= 2*M_PI;
        }
        return temp - M_PI;
    }
    else{
        while (temp < 0){
            temp += 2*M_PI;
        }
        return temp - M_PI;
    }
}
