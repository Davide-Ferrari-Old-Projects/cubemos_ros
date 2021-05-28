#include "QuaternionAlgebra.h"

MatrixXd QuaternionAlgebra::quatCon(MatrixXd q)
{

	MatrixXd qCon(4,1);
	qCon(0, 0) = q(0, 0);
	qCon(1, 0) = -q(1, 0);
	qCon(2, 0) = -q(2, 0);
	qCon(3, 0) = -q(3, 0);

	return qCon;
}

MatrixXd QuaternionAlgebra::quatProd(MatrixXd q, MatrixXd w)
{

	MatrixXd qProd(4, 1);
	qProd(0, 0) = q(0, 0)*w(0, 0) - q(1, 0)*w(1, 0) - q(2, 0)*w(2, 0) - q(3, 0)*w(3, 0);
	qProd(1, 0) = q(1, 0)*w(0, 0) + q(0, 0)*w(1, 0) - q(3, 0)*w(2, 0) + q(2, 0)*w(3, 0);
	qProd(2, 0) = q(2, 0)*w(0, 0) + q(3, 0)*w(1, 0) + q(0, 0)*w(2, 0) - q(1, 0)*w(3, 0);
	qProd(3, 0) = q(3, 0)*w(0, 0) - q(2, 0)*w(1, 0) + q(1, 0)*w(2, 0) + q(0, 0)*w(3, 0);

	return qProd;
}

MatrixXd QuaternionAlgebra::exp_q(MatrixXd q)
{

	MatrixXd e_q(4, 1);
	MatrixXd qv(3, 1);
	qv << q(1, 0), q(2, 0), q(3, 0);



	double norm_qv = sqrt(q(1, 0)*q(1, 0) + q(2, 0)*q(2, 0) + q(3, 0)*q(3, 0));

	if ((norm_qv) > 1e-12)
	{
		e_q(0, 0) = exp(q(0, 0))*cos(norm_qv);
		e_q.block<3, 1>(1, 0) = qv / norm_qv*sin(norm_qv);
	}
	else
	{
		e_q(0, 0) = 1.0;
		e_q(1, 0) = 0.0;
		e_q(2, 0) = 0.0;
		e_q(3, 0) = 0.0;

	}

	return e_q;

}

MatrixXd QuaternionAlgebra::exp_q_pure(MatrixXd v)
{

	MatrixXd e_q(4, 1);
	MatrixXd qv(3, 1);
	qv << v(0, 0), v(1, 0), v(2, 0);



	double norm_qv = sqrt(v(1, 0)*v(1, 0) + v(2, 0)*v(2, 0) + v(0, 0)*v(0, 0));

	if ((norm_qv) > 1e-12)
	{
		e_q(0, 0) = cos(norm_qv);
		e_q.block<3, 1>(1, 0) = qv / norm_qv*sin(norm_qv);
	}
	else
	{
		e_q(0, 0) = 1.0;
		e_q(1, 0) = 0.0;
		e_q(2, 0) = 0.0;
		e_q(3, 0) = 0.0;

	}

	return e_q;

}

MatrixXd QuaternionAlgebra::worldToBody(MatrixXd v, MatrixXd q)
{
	MatrixXd v_quat(4, 1);
	MatrixXd v_body_quat(4, 1);
	MatrixXd v_body(3, 1);

	v_quat(0, 0) = 0;
	v_quat.block<3, 1>(1, 0) = v;

	v_body_quat = quatProd(quatProd(quatCon(q), v_quat), q);
	v_body = v_body_quat.block<3, 1>(1, 0);

	return v_body;
}

MatrixXd QuaternionAlgebra::bodyToWorld(MatrixXd v, MatrixXd q)
{
	MatrixXd v_quat(4, 1);
	MatrixXd v_world_quat(4, 1);
	MatrixXd v_world(3, 1);

	v_quat(0, 0) = 0;
	v_quat.block<3, 1>(1, 0) = v;

	v_world_quat = quatProd(quatProd(q, v_quat), quatCon(q));
	v_world = v_world_quat.block<3, 1>(1, 0);

	return v_world;
}

MatrixXd QuaternionAlgebra::removeGravityFormAcc(MatrixXd a, MatrixXd q,double g)
{
	
	MatrixXd a_nograv(3, 1);
	MatrixXd G = MatrixXd::Zero(3, 1);
	// set G
	G(2, 0) = g;

	

	a_nograv = a - worldToBody(G, q);
	

	return a_nograv;
}
