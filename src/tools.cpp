#include <iostream>
#include "tools.h"
#include <math.h>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	int estimationsSize = estimations.size();

	// check that the estimation vector size should not be zero and they are equal
	if(estimationsSize == 0 || estimationsSize != ground_truth.size()){
	    cout << "AH, THEY WEREN'T THE RIGHT SIZE" << endl;
	    return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimationsSize; ++i){
        cout << "estimations[" << i << "]: " << estimations[i] << endl;
        cout << "ground_truth[" << i << "]: " << ground_truth[i] << endl;

        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array().square();

        rmse += residual;
        cout << "step: " << i << endl;
        cout << "rmse: " << rmse << endl;
	}

	// calculate the mean
    rmse = rmse.array()/estimationsSize;

	// calculate the squared root
	rmse = rmse.array().sqrt();

    // return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	cout << "Calculate Jacobian..." << endl;
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float px2py2 = (px * px) + (py * py);

	//check division by zero
	if(fabs(px2py2) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << px/sqrt(px2py2), py/sqrt(px2py2), 0, 0,
	      -py/px2py2, px/px2py2, 0, 0,
	      (py*(vx*py-vy*px))/pow(px2py2, 1.5), (px*(vy*px-vx*py))/pow(px2py2, 1.5), px/sqrt(px2py2), py/sqrt(px2py2);

	return Hj;
}

float Tools::NormalizeAngle(float angle){
    cout << "Normalize angle..." << endl;
    float TWO_PI = 2.0*M_PI;

    angle = fmod(angle + M_PI, TWO_PI);

    if (angle < 0)
        angle += TWO_PI;

    return angle - M_PI;
}
