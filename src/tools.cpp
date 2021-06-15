#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    if(estimations.empty() || estimations.size()!=ground_truth.size())
        return rmse;
    for (int i=0; i < estimations.size(); ++i) {
        VectorXd diff=estimations[i]-ground_truth[i];
        diff =(diff.array() * diff.array());
        rmse+=diff;

    }
    rmse = rmse * 1/estimations.size();
    rmse=rmse.cwiseSqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);
    Hj=Hj.setZero();
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float psqr = hypot(px,py);
    float padd = px*px+py*py;
    float pcub = padd*psqr;

    if(fabs(padd)<0.0001)
    {
        std::cout << "Divide by zero" << std::endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj(0,0) = px / psqr;                        Hj(0,1) = py / psqr;
    Hj(1,0) = - (py/padd);                      Hj(1,1) = (px/padd);
    Hj(2,0) = ((py * (vx*py - vy*px))/pcub);    Hj(2,1) = ((px * (vy*px - vx*py))/pcub);
    Hj(2,2) = px / psqr;
    Hj(2,3) = py / psqr;
    return Hj;
}

Eigen::VectorXd Tools::CartisenToPolar(const Eigen::VectorXd& x_state) {
    float px=x_state[0];
    float py=x_state[1];
    float vx=x_state[2];
    float vy=x_state[3];
    float rho= hypot(px,py);
    float phi=atan2(py, px);
    float rhodot = 0;
    if (fabs(rho)>0.0001) {
        rhodot = (px * vx + py * vy) / rho;
    }
    Eigen::VectorXd new_x_state=VectorXd(3);
    new_x_state << rho, phi, rhodot;
    return new_x_state;
}

float Tools::normalizePhi(float phi) {
    if (phi>M_PI){
        return normalizePhi(phi-2*M_PI);
    }
    if(phi<-M_PI){
        return normalizePhi(phi+2*M_PI);
    }
    return phi;
}
