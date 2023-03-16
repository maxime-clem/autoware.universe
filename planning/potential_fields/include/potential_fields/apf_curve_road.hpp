#include <iostream>
#include <cmath>
#include <vector>
#include <stdio.h>
using namespace std;

typedef std::vector<double> MyVec;

// global parameters
const double A_r = 0.5; // road PF depth
const double A_o = 1;
const double b_r = 1; // para. controlling road PF width
const double c2 = 5e-6; // polynomial coefficients of curve road
const double c1 = 5e-8; // polynomial coefficients of curve road
const double c0_rlc = 0; // polynomial coefficients of curve road: upper or left bound
const double c0_llc = 3.5; // polynomial coefficients of curve road: lower or right bound
const double delta_Xh = 1e-10; // additional term in case infinity
const double sigma_y = 0.865; // simplify the lateral calc.

int sign(float x)
{
	if(x>0)
		return 1;
	else if (x == 0)
		return 0;
	else
		return -1;
}

float sqrt(float x)
{
    float xhalf = 0.5f*x;
    int i = *(int*)&x; // get bits for floating VALUE 
    i = 0x5f375a86- (i>>1); // gives initial guess y0
    x = *(float*)&i; // convert bits BACK to float
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    return x;
}  

double getRoadPF(double Ego_info[]){
    // Ego_info[7]={X_ego,Y_ego,Ve_lat,Ve_long,Ae_lat,Ae_long,Yaw_ego,M_ego,Le_l,Le_w}
	// extract info
    double X_ego = Ego_info[0];
    double Y_ego = Ego_info[1];
    // right lane
	double m_y = -1/(2*c2*(X_ego + delta_Xh) + c1);
	double Y_rlc_cr =  c2*pow(X_ego + delta_Xh,2) + c1*(X_ego + delta_Xh) + c0_rlc;
	double b_y = Y_rlc_cr - m_y*(X_ego + delta_Xh);
	double U_rl_cr = A_r*pow(1 - exp( -b_r*sign(Y_ego - Y_rlc_cr)*sqrt( pow((Y_ego - b_y)/m_y - \
                     (X_ego + delta_Xh),2) + pow(Y_rlc_cr - Y_ego,2) ) ),2);
    // left lane
    double Y_llc_cr = c2*pow(X_ego + delta_Xh,2) + c1*(X_ego + delta_Xh) + c0_llc;
    double b_y_ll = Y_llc_cr - m_y*(X_ego + delta_Xh);
    double U_ll_cr = A_r*pow(1 - exp( b_r*sign(Y_ego - Y_llc_cr)*sqrt( pow((Y_ego-b_y_ll)/m_y - \
                      (X_ego + delta_Xh),2) + pow(Y_llc_cr - Y_ego,2) ) ),2);
    // total 
    double U_cr = U_rl_cr + U_ll_cr;

    return U_cr;
}

double getObsPF(double Ego_info[], double Obs_info[]){
    // extract info
    // ego vehicle
    double X_ego = Ego_info[0];
    double Y_ego = Ego_info[1];
    double Ve_lat = Ego_info[2];
    double Ve_long = Ego_info[3];
    double Ae_lat = Ego_info[4];
    double Ae_long = Ego_info[5];
    double Yaw_ego = Ego_info[6];
    double M_ego = Ego_info[7];
    double Le_l = Ego_info[8];
    double Le_w = Ego_info[9];

    // obstacle vehicle
    double X_obs = Obs_info[0];
    double Y_obs = Obs_info[1];
    double Vo_lat = Obs_info[2];
    double Vo_long = Obs_info[3];
    double Ao_lat = Obs_info[4];
    double Ao_long = Obs_info[5];
    double Yaw_obs = Obs_info[6];
    double M_obs = Obs_info[7];
    double Lo_l = Obs_info[8];
    double Lo_w = Obs_info[9];

    // basic parameter setting
    int F_m = 300; // brake force of each tire
    double x_sigma = (M_ego*(Ve_long*Ve_long-Vo_long*Vo_long))/(8*F_m)+(Le_l+Lo_l)/2;

    // obstacle PF
    double sigma_x = sqrt(-pow(x_sigma,2)/(2*log(0.01/A_o)));  
    double psi_obs_tem =  atan(2*c2*X_obs + c1);// estimated value if ego can't get the obstacle's yaw info
    double a_psi = pow(cos(psi_obs_tem),2)/(2*sigma_x*sigma_x)+pow(sin(psi_obs_tem),2)/(2*sigma_y*sigma_y);
    double b_psi = -(sin(2*psi_obs_tem)/(4*sigma_x*sigma_x)+sin(2*psi_obs_tem)/(4*sigma_y*sigma_y));
    double c_psi = pow(sin(psi_obs_tem),2)/(2*pow(sigma_x,2))+pow(cos(psi_obs_tem),2)/(2*pow(sigma_y,2));
    double U_obs = A_o*exp(-(a_psi*pow(X_ego - X_obs,2) + 2*b_psi*(X_ego - X_obs)*(Y_ego - \
                        Y_obs) + c_psi*pow(Y_ego - Y_obs,2)));
    return U_obs;
}

MyVec Fren2Cart(double s, double s_dot, double s_dot2, double l, double l_dot, double l_dot2, \
                 double ref_path[]){
    // extract reference info
    double X_r = ref_path[0];
    double Y_r = ref_path[1];
    double Theta_r = ref_path[2];
    double Kappa_r = ref_path[3];
    double Kappa_r_dot = ref_path[4];

    // calculate cartesian frame
    MyVec Carte_info;
    double X = X_r - l*sin(Theta_r);
    double Y = Y_r - l*cos(Theta_r);
    double Theta = atan(l_dot/(1-Kappa_r*l)) + Theta_r;
    double V = sqrt(pow(s_dot*(1-Kappa_r*l),2) + pow(s_dot*l_dot,2));
    double Kx = ((l_dot2+(Kappa_r_dot*l+Kappa_r*l_dot)*tan(Theta-Theta_r))*pow(cos(Theta-\
                Theta_r),2)/(1-Kappa_r*l) + Kappa_r)*cos(Theta-Theta_r)/(1-Kappa_r*l);
    double Ax = s_dot2*(1-Kappa_r*l)/cos(Theta-Theta_r) + (pow(s_dot,2)/cos(Theta-Theta_r))*(l_dot*(Kx*(1- \
                Kappa_r*l)/cos(Theta-Theta_r) - Kappa_r) - (Kappa_r_dot*l+Kappa_r*l_dot));
    // return the info in Cartesian coordinate
    Carte_info[0] = X;
    Carte_info[1] = Y;
    Carte_info[2] = Theta;
    Carte_info[3] = V;
    Carte_info[4] = Kx;
    Carte_info[5] = Ax;
    return Carte_info;
}

int main() 
{
    cout <<pow(64.5,2)<<endl;
    return 0;
}