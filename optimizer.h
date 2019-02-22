#pragma once
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

//g2o
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
//#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"


class Optimizer
{
public:
	enum solverType
	{
		DENSE,
		CHOLMOD,
		CSPARSE
	};

	Optimizer();
	void BundleAdjustment(const std::vector<cv::Mat>& T_c_ws, const std::vector<cv::Point3f>& MapPoints, 
						const std::vector<std::vector<cv::Point2d>> vertexss, const solverType& sType, const cv::Mat& K);

private:





};
#endif //OPTIMIZER_H