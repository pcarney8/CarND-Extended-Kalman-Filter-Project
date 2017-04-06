#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	//TODO: make sure this 4 variable vector is accurate, is it our state vector with [px, py, vx, vy]?
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	int estimationsSize = estimations.size();
 	cout << estimationsSize << endl;
 	cout << ground_truth.size() << endl;

	if(estimationsSize == 0 || estimationsSize != ground_truth.size()){
	    //should this return null? or just throw an error? or skip?
	    cout << "AH, THEY WEREN'T THE RIGHT SIZE" << endl;
	    return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimationsSize; ++i){
        cout << "estimations[" << i << "]: " << estimations[i] << endl;
        cout << "ground_truth[" << i << "]: " << ground_truth[i] << endl;

        VectorXd residual = estimations[i] - ground_truth[i];

        cout << "after sub, residual: " << residual << endl;

        residual = residual.array().square();
        cout << "after square, residual: " << residual << endl;

        // not compressing, apparently these are 4 measurments taken at the same time?
        // rmse[i] = residual.sum();

        rmse += residual;
        cout << "step: " << i << endl;
        cout << "rmse: " << rmse << endl;
	}


	//calculate the mean
    rmse = rmse.array()/estimationsSize;
 	cout << "mean, rmse: " << rmse << endl;

	//calculate the squared root
	rmse = rmse.array().sqrt();
    cout << "sqrt, rmse: " << rmse << endl;

//	return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float px2py2 = (px * px) + (py * py);
	float c2 = sqrt(px2py2);
	float c3 = (px2py2*c2);

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
