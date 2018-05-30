#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
  try
  {
    assert(estimations.size() != 0);
  }
  catch (...)
  {
    std::cout << "Estimations has size zero \n" << estimations.size() << std::endl;
  }
	
	//  * the estimation vector size should equal ground truth vector size
  try
  {
    assert(estimations.size() == ground_truth.size());
  }
  catch (...)
  {
    std::cout << "Estimations different size to truth \n" << ground_truth .size() << std::endl;
  }
  
	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

  //calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}