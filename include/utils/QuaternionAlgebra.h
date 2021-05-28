#ifndef QUATERNIONALGEBRA_H
#define QUATERNIONALGEBRA_H

#include <Eigen/Geometry>

using namespace Eigen;
class QuaternionAlgebra
{
public:
	static MatrixXd quatProd(MatrixXd q, MatrixXd w);
	static MatrixXd quatCon(MatrixXd q);
	static MatrixXd exp_q(MatrixXd q);
	static MatrixXd exp_q_pure(MatrixXd v);

	static MatrixXd bodyToWorld(MatrixXd v,MatrixXd q);
	static MatrixXd worldToBody(MatrixXd v,MatrixXd q);

	static MatrixXd removeGravityFormAcc(MatrixXd a, MatrixXd q,double g);

private:
	// Disallow creating an instance of this object
	QuaternionAlgebra() {}
};

#endif