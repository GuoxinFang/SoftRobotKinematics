#include "stdafx.h"
#include "soroPneumaticKinematics.h"
#include <omp.h>
#include <iostream>
#include <fstream>

#define VELE 4
#define OMPCORE 32
#define Square(x) ((x)*(x))

using namespace std;
using namespace Eigen;


soroPneumaticKinematics::soroPneumaticKinematics(QMeshPatch* inputMesh, int maxIterTime) {

	// ------------------------------
	// initialize matrix N, c1 and c2

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (i == j) N(i, j) = 0.75;
			else N(i, j) = -0.25;
		}
	}

	model = inputMesh;
	maxIter = maxIterTime;

}

void soroPneumaticKinematics::buildSimulationSystem() {

	/* Initialize parameters */

	select_num = 0; // this is the node number in selection region
	EachCore = model->GetTetraNumber() / OMPCORE + 1;

	eleNum = model->GetTetraNumber();
	nodeNum = model->GetNodeNumber();

	model->initializeListIndex();

	/* Initialize matrix */

	nodePos.resize(nodeNum, 3); // current node position
	nodePosInit.resize(nodeNum, 3); // initial node position

	tetTable = Eigen::MatrixXd::Zero(eleNum, 4); // topology relationship
	tetSelected = Eigen::VectorXd::Zero(eleNum); // install the chamber element
	weight = Eigen::VectorXd(eleNum); // install weighting for every tetrahedral

	shiftTable = Eigen::MatrixXi::Zero(nodeNum, 3);	// flag: ICP region select, top select, bottom select

	/* Fill data to matrix */

	for (GLKPOSITION Pos = model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)model->GetNodeList().GetNext(Pos);
		int index = node->GetIndexNo();
		if (node->shirftSelect == true) {
			shiftTable(index, 0) = 1;
			select_num++;
		}
		Vector3d pos;  node->GetCoord3D(pos); nodePos.row(node->GetIndexNo()) = pos;
		Vector3d posInit;  node->SetCoord3D_last(pos(0), pos(1), pos(2));
		node->GetCoord3D_last(posInit(0), posInit(1), posInit(2)); nodePosInit.row(node->GetIndexNo()) = posInit;
	}

	// Preparing for the shifting matrix
	double highYCoordinate = -999999.0;
	double lowYCoordinate = 999999.0;
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (nodePosInit(i, 1) > highYCoordinate) highYCoordinate = nodePosInit(i, 1);
		if (nodePosInit(i, 1) < lowYCoordinate) lowYCoordinate = nodePosInit(i, 1);
	}
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (highYCoordinate - nodePosInit(i, 1) < 0.05)  shiftTable(i, 1) = 1;
		if (nodePosInit(i, 1) - lowYCoordinate < 0.05) shiftTable(i, 2) = 1;
	}

	shiftInit = Eigen::MatrixXd::Constant(select_num, 3, 1.0);
	shiftPosInit = Eigen::Vector3d::Zero(); int selectIndex = 0;
	for (GLKPOSITION Pos = model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)model->GetNodeList().GetNext(Pos);
		if (node->shirftSelect == true) {
			node->GetCoord3D_last(shiftInit(selectIndex, 0), shiftInit(selectIndex, 1), shiftInit(selectIndex, 2));
			for (int i = 0; i < 3; i++) shiftPosInit[i] += shiftInit(selectIndex, i) / select_num;
			selectIndex++;
		}
	}

	// build topology, install chamber selection and weight for all elements
	for (GLKPOSITION Pos = model->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* tetra = (QMeshTetra*)model->GetTetraList().GetNext(Pos);
		int tetindex = tetra->GetIndexNo();
		for (int i = 0; i < 4; i++) tetTable(tetindex, i) = tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		for (int chamberIndex = 0; chamberIndex < 3; chamberIndex++) {
			if (tetra->isChamber[chamberIndex] == true) { tetSelected(tetindex) = expandRatio[chamberIndex]; break; }
		}

		if (tetra->isRigid) weight(tetindex) = weightHard; // hard material region
		else if (tetSelected(tetindex) > 0.0) weight(tetindex) = weightChamber; // chamber region
		else weight(tetindex) = 1.0;

		//if (weight(tetindex) > 3.1) 
			//std::cout << "this tetra weight > 0, index = " << tetindex << ", weight = " << weight(tetindex) << std::endl;
	}

	// ------------------------------------------------
	// allocate memory for SVD in GPU
	/*buildGPUMemory_forSVD(model->GetTetraNumber());
	SVDInputMatrix = (double*)malloc(model->GetTetraNumber() * 9 * sizeof(double));
	SVDOutputMatrix = (double*)malloc(model->GetTetraNumber() * 9 * sizeof(double));*/

	LocalCoord.resize(eleNum);
	InverseP.resize(eleNum);
	LocalGoal.resize(eleNum);

	VectorXPosition.resize(3);
	VectorBSide.resize(3);
	VectorXPosition_Last.resize(3);

	for (int i = 0; i < 3; i++) {
		VectorXPosition[i] = Eigen::VectorXd::Zero(nodeNum);
		VectorXPosition_Last[i] = Eigen::VectorXd::Zero(nodeNum);
		VectorBSide[i] = Eigen::VectorXd::Zero(eleNum * VELE);
	}
	centerTop = Eigen::MatrixXd::Zero(500, 3);


	tipPosInit = _shifttoOriginalPosAA();

	// ----------------------------------------------------------------------
	// build localGoal and InverseP matrix - 
	// only for given tetSelected table (chamber expanding ratio)

	_compLocalGoalandInverse();

	// ------------------------------------------------
	// build matrix A, AT, ATA and can be factorized in advance

	_buildMatA();

	// ------------------------------------------------
	// build system for AA acceleration
	buildSystemAA(maxIter + 1, false);

}

void soroPneumaticKinematics::buildSystemAA(int iterTime, bool IKCompute) {

	/* Initialize the parameter */
	Eperv = 99999999.9999;
	energyAAIter.clear();
	AAStopFlag = false;
	afterStopIterNum = 0;

	AA_Qset.resize(iterTime + 1);
	AA_Fset.resize(iterTime + 1);
	AA_Gset.resize(iterTime + 1);

	for (int i = 0; i < iterTime + 1; i++) {
		AA_Qset[i] = Eigen::MatrixXd::Zero(nodeNum, 3);
		AA_Fset[i] = Eigen::MatrixXd::Zero(nodeNum, 3);
		AA_Gset[i] = Eigen::MatrixXd::Zero(nodeNum, 3);
	}

	Q_LG = Eigen::MatrixXd::Zero(nodeNum, 3);
	iterQ = Eigen::MatrixXd::Zero(nodeNum, 3);
	P_AA.resize(eleNum);
	for (int i = 0; i < eleNum; i++) P_AA[i] = Eigen::MatrixXd::Zero(3, 4);

	/* Begin the first iteration of the forward kinematics */

	//_updateQ(iterQ);	
	if (IKCompute == false) {
		iterQ = nodePosInit; /* Method 1: Restore the system! */
		//iterQ = nodePos; /* Method 2: Using the position from last iteration as initial guess! */
		nodePos = nodePosInit;

	}
	else {
		//nodePos = FKPos;
		nodePos = nodePosInit;
	}

	AA_Qset[0] = iterQ; //Q0

	_localStepAA(AA_Qset[0], P_AA);
	double initEnergyAA = _calEnergyIterAA(AA_Qset[0], P_AA);
	// cout.precision(17); std::cout << "iter " << 0 << " energy = " << initEnergyAA << std::endl;
	energyAAIter.push_back(initEnergyAA);

	_globalStepAA(P_AA, AA_Qset[1]); //run first iteration to compute Q1

	AA_Gset[0] = AA_Qset[1];
	AA_Fset[0] = AA_Gset[0] - AA_Qset[0]; // F0
}

/* Function build for final usage Sept-12-2020 */

void soroPneumaticKinematics::computeInverseKinematics() {

	int terminalValue = 0.1; // unit (mm)

	Eigen::Vector3d targetPos, IKPara;
	targetPos << 43.9915, 119.316, 21.4531;
	IKPara << 1.45, 1.75, 1.3;

	Eigen::Vector3d currPara;
	Eigen::Vector3d currentPos;
	double objectiveInit;
	double ratio = 0.2;

	int maxiterTime = 1000;

	do {
		printf("--------------------------------\n");

		//double step = 1.0; 

		/* compute objective and gradient for this iteration */
		std::cout << " init IKPara = " << IKPara(0) << ", "
			<< IKPara(1) << ", " << IKPara(2) << std::endl;

		this->updateExpandRatio(IKPara);
		currentPos = this->initSolverSingle(maxiterTime, true);

		std::cout << " init tip Pos estimated by network = " << currentPos(0) << ", "
			<< currentPos(1) << ", " << currentPos(2) << std::endl;

		std::cout << " differ (t-Pc) = " << (targetPos - currentPos)(0) << ", "
			<< (targetPos - currentPos)(1) << ", " << (targetPos - currentPos)(2) << std::endl;

		objectiveInit = Square((targetPos - currentPos).norm());

		std::cout << "objectiveInit = " << objectiveInit << std::endl;

		/* compute the initial search length */
		double step = pow(3 * sqrt(objectiveInit) * pow(10, -5), 1.0 / 3);
		std::cout << "this iter step = " << step << std::endl;


		/* Compute gradient */
		std::vector<Eigen::Vector3d> tipPos(6);
		for (int i = 0; i < 6; i++)
			tipPos[i] = Eigen::Vector3d::Zero();
		Eigen::MatrixXd expandRatioGradient(6, 3);
		expandRatioGradient << 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1;
		for (int chamberNum = 0; chamberNum < 3; chamberNum++) {
			for (int i = 0; i < 6; i++) {
				expandRatioGradient(i, chamberNum) =
					expandRatioGradient(i, chamberNum) * step + IKPara(chamberNum);
			}
		}
		std::cout << expandRatioGradient << std::endl;

		for (int i = 0; i < 6; i++) {
			Eigen::Vector3d actuationParaGradient;
			for (int chamberNum = 0; chamberNum < 3; chamberNum++)
				actuationParaGradient(chamberNum) = expandRatioGradient(i, chamberNum);
			this->updateExpandRatio(actuationParaGradient);

			tipPos[i] = this->initSolverSingle(maxiterTime, true);
			//tipPos.row(i + 1) = this->hyberSolverSingle(maxiterTime, true);
		}

		for (int i = 0; i < 6; i++)
			std::cout << "tipPos = \n" << tipPos[i](0) << ", " << tipPos[i](1) << ", " << tipPos[i](2) << std::endl;

		/* use eq.(1) to compute the gradient */
		Eigen::Vector3d gradient_EQ1;
		for (int i = 0; i < 3; i++) {
			Eigen::VectorXd differ1 = targetPos - tipPos[2 * i];
			Eigen::VectorXd differ2 = targetPos - tipPos[2 * i + 1];
			gradient_EQ1(i) = (Square(differ1.norm()) - Square(differ2.norm())) / (2 * step);
		}

		/* use eq.(3) to compute the gradient */
		Eigen::Vector3d gradient_EQ3 = Eigen::Vector3d::Zero();
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
			gradient_EQ3(i) += (-2.0) * ((targetPos - currentPos)(j)) * ((tipPos[2 * i] - tipPos[2 * i + 1])(j)) / (2 * step);

		Eigen::Matrix3d JacobianMat;
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
			JacobianMat(i, j) = ((tipPos[2 * i] - tipPos[2 * i + 1])(j)) / (2 * step);

		std::cout << " method 1 ----- " << gradient_EQ1(0) << ", " << gradient_EQ1(1) << ", " << gradient_EQ1(2) << std::endl;
		std::cout << " method 3 ----- " << gradient_EQ3(0) << ", " << gradient_EQ3(1) << ", " << gradient_EQ3(2) << std::endl;

		std::cout << " method 3 ----- Jacobian Mat\n " << JacobianMat << std::endl;

		Eigen::Vector3d currentObjectiveGradient = gradient_EQ1;

		/*shrink step*/
		printf("\n ---------- shrink step begin!\n");
		double currentObjective;
		do
		{
			step *= ratio;

			currPara = IKPara - step * currentObjectiveGradient;
			std::cout << "currPara = " << currPara(0) << ", " << currPara(1) << ", " << currPara(2) << std::endl;

			while (currPara(0) < 1 || currPara(0) > 3.5 || currPara(1) < 1 || currPara(1) > 3.5 || currPara(2) < 1 || currPara(2) > 3.5)
			{
				step *= ratio;
				currPara = IKPara - step * currentObjectiveGradient;
			}

			this->updateExpandRatio(currPara);
			Eigen::Vector3d thisPos = this->initSolverSingle(maxiterTime, true);

			//std::cout << "thisPos = " << thisPos << std::endl;

			currentObjective = Square((targetPos - thisPos).norm());
			std::cout << " ** shrink Objective = " << currentObjective << std::endl;

		} while (currentObjective > objectiveInit);

		std::cout << "\n ratio = " << step << std::endl;
		/*expand step*/
		printf("expand step begin!\n");
		for (int i = 1; i < 10; i++) {
			std::cout << " ** expand para = " << currPara(0) << ", "
				<< currPara(1) << ", " << currPara(2) << std::endl;

			currPara -= step * currentObjectiveGradient;

			for (int i = 0; i < 3; i++) {
				if (currPara(i) < 1) currPara(i) = 1;
				if (currPara(i) > 3.5) currPara(i) = 3.5;
			}

			Eigen::Vector3d thisPos;

			this->updateExpandRatio(currPara);
			thisPos = this->initSolverSingle(maxiterTime, true);

			double thisObjective = Square((targetPos - thisPos).norm());
			std::cout << "      expand Objective = " << thisObjective << std::endl << std::endl;

			if (thisObjective > currentObjective) {
				currPara += step * currentObjectiveGradient;
				break;
			}
			currentObjective = thisObjective;
		}
		IKPara = currPara;
		printf("--------------------------------\n\n\n");

	} while (objectiveInit > terminalValue);

	std::cout<< "IK solution: actuation parameter = \n" <<
		IKPara(0) << ", " << IKPara(1) << ", " << IKPara(2) << std::endl;
}

Eigen::Vector3d& soroPneumaticKinematics::hyberSolverSingle(int maxiterTime, bool gradientFK) {

	this->buildSystemAA(maxiterTime, gradientFK);

	/* Simulation for the first point */
	bool startNormalIterFlag = false;
	int normalIterTime = 0;
	for (int i = 0; i < maxiterTime; i++) {
		/* First using Anderson solver until the energy increase */
		if (startNormalIterFlag == false) {
			if (this->andersonSolver(i)) {
				startNormalIterFlag = true;
			}
		}
		else {
			/* Using normal solver until the tip position not significantly change */
			if (normalIterTime < 10) {
				this->initSolver(i, false, gradientFK);
			}
			else {
				if (this->initSolver(i, true, gradientFK)) {
					if (gradientFK == false) this->saveCurrentPos();
					std::cout << " -- iteration stop at iter# " << i << std::endl << std::endl;
					break;
				}
			}
			normalIterTime++;
		}
	}

	return tipPosInit;

}

Eigen::Vector3d& soroPneumaticKinematics::initSolverSingle(int maxiterTime, bool gradientFK) {
	this->buildSystemAA(maxiterTime, gradientFK);

	/* Simulation for the first point */
	int normalIterTime = 0;
	for (int i = 0; i < maxiterTime; i++) {
		/* Using normal solver until the tip position not significantly change */
		if (normalIterTime < 10) {
			this->initSolver(i, false, gradientFK);
		}
		else {
			if (this->initSolver(i, true, gradientFK)) {
				if (gradientFK == false) this->saveCurrentPos();
				std::cout << "iteration stop at iter# " << i << std::endl;
				break;
			}
		}
		normalIterTime++;

	}
	return tipPosInit;
}

bool soroPneumaticKinematics::initSolver(int iterTime, bool checkTerminal, bool gradientFK) {

	/*std::cout << " before iter, tip pos = " << tipPosInit(0) << ", " <<
		tipPosInit(1) << ", " << tipPosInit(2) << std::endl;*/

	_localStepAA(nodePos, P_AA);	// local step
	_globalStepAA(P_AA, nodePos);
	Eigen::Vector3d tipPosCurrent = _shifttoOriginalPosAA();

	Eperv = _calEnergyIterAA(nodePos, P_AA);
	energyAAIter.push_back(Eperv);

	bool stopIterFlag = false;
	double iterGap = (tipPosCurrent - tipPosInit).norm();
	//std::cout << "iter " << iterTime << ", gap = " << iterGap << std::endl;

	tipPosInit = tipPosCurrent;

	/* notice that only after few iteration of initial solver the system will make terminal checking */
	double terminalValueThisIter = terminalValue;
	int gradientFKPara = 1; // for gradient FK computing, this can be adjusted to compute more precise Jacobian matrix
	if (gradientFK) terminalValueThisIter /= gradientFKPara;

	if (iterGap < sqrt(terminalValueThisIter) && checkTerminal)
		stopIterFlag = true;

	return stopIterFlag;

}

bool soroPneumaticKinematics::andersonSolver(int iterTime) {

	int k = iterTime + 1;

	// -- local step
	_localStepAA(AA_Qset[k], P_AA);
	double energyIterInit = _calEnergyIterAA(AA_Qset[k], P_AA);

	if (energyIterInit > Eperv + eps) {
		//std::cout << "  iter - " << k << " previous energy is lower as " << Eperv << "And QSet K gives" << energyIterInit << std::endl;
		AA_Qset[k] = Q_LG;
		_localStepAA(Q_LG, P_AA);
		double E_LG = _calEnergyIterAA(AA_Qset[k], P_AA);

		/* stop AA step if the energy is increase! */
		if (Eperv < E_LG) {
			afterStopIterNum++;
			if (afterStopIterNum == maxVariationIterNum) {
				nodePos = AA_Qset[k - 1];
				printf(" -- AA stop flag is activated at iteration %d. \n\n", k);
				return true;
			}
		}

		Eperv = E_LG;
	}
	else Eperv = energyIterInit;
	energyAAIter.push_back(Eperv);

	// -- global step
	_globalStepAA(P_AA, Q_LG);

	// -- AA step

	AA_Gset[k] = Q_LG;
	AA_Fset[k] = AA_Gset[k] - AA_Qset[k];
	int mk; if (k < m) mk = k; else mk = m;

	Eigen::MatrixXd D(3 * nodeNum, mk);	Eigen::VectorXd FkFlat(3 * nodeNum);
	for (int j = 0; j < mk; j++) {
		Eigen::MatrixXd deltaF_kj = AA_Fset[k - j] - AA_Fset[k - j - 1]; // delta F(k-j)
		for (int row = 0; row < nodeNum; row++) {
			for (int coord = 0; coord < 3; coord++) {
				D(row + nodeNum * coord, j) = deltaF_kj(row, coord); // make x,y,z-coordinate flat
				FkFlat(row + nodeNum * coord) = AA_Fset[k](row, coord);
			}

		}
	}

	Eigen::MatrixXd DT = D.transpose();
	Eigen::VectorXd theta = ((DT * D).inverse()) * (DT * FkFlat); // std::cout << theta << std::endl;

	AA_Qset[k + 1] = Q_LG;
	for (int j = 0; j < mk; j++) AA_Qset[k + 1] -= theta(j) * (AA_Gset[k - j] - AA_Gset[k - j - 1]);

	return false;
}

Eigen::Vector3d& soroPneumaticKinematics::getTipPos() {
	return tipPosInit;
}

bool soroPneumaticKinematics::solver(int iterTime, bool accelerate) {

	// the first iteration is already computed in "buildSystemAA" function
	int k = iterTime + 1;

	if (accelerate == false) {
		_localStepAA(AA_Qset[k], P_AA);	// local step

		Eperv = _calEnergyIterAA(AA_Qset[k], P_AA); // compute energy
		energyAAIter.push_back(Eperv);
		std::cout << "iter " << k << " energy = " << Eperv << std::endl;

		_globalStepAA(P_AA, AA_Qset[k + 1]); // global step

		nodePos = AA_Qset[k + 1];
		_shifttoOriginalPosAA();
		return true;
	}
	else {

		// -- local step
		long totaltime = clock();
		_localStepAA(AA_Qset[k], P_AA);
		//printf(" local step time = %ld ms.\n", clock() - totaltime);

		double energyIterInit = _calEnergyIterAA(AA_Qset[k], P_AA);

		if (energyIterInit > Eperv + eps) {
			// std::cout << "  iter - " << k << " perv energy is lower! perv is " << Eperv << "And QSet K gives" << energyIterInit <<  std::endl;
			AA_Qset[k] = Q_LG;
			_localStepAA(Q_LG, P_AA);

			double E_LG = _calEnergyIterAA(AA_Qset[k], P_AA);

			if (Eperv < E_LG && k>10) {
				nodePos = AA_Qset[k - 1];
				_shifttoOriginalPosAA(); return false;
			}
			else Eperv = _calEnergyIterAA(AA_Qset[k], P_AA);
		}
		else Eperv = energyIterInit;

		cout.precision(17); std::cout << "iter " << k << " energy = " << Eperv << std::endl;
		energyAAIter.push_back(Eperv);

		//_shifttoInitialPos(); // update the mesh based on last iteration

		// determine if the iteration need to continue
		//nodePos = AA_Qset[k];
		//centerTop.row(k) = _shifttoOriginalPosAA();
		//if (k >= 100 && pow(((centerTop.row(k) - centerTop.row(k - 1)).norm()), 2.0) < terminalValue) {
		//	std::cout << " -- Converge! iter Time = " << k << std::endl; return false;
		//}

		//----------------------------
		// AA begin

		//totaltime = clock();

		_globalStepAA(P_AA, Q_LG);
		AA_Gset[k] = Q_LG; AA_Fset[k] = AA_Gset[k] - AA_Qset[k];

		//printf(" global step time = %ld ms.\n", clock() - totaltime);

		//totaltime = clock();

		int mk; if (k < m) mk = k; else mk = m;

		Eigen::MatrixXd D(3 * nodeNum, mk);	Eigen::VectorXd FkFlat(3 * nodeNum);
		for (int j = 0; j < mk; j++) {
			Eigen::MatrixXd deltaF_kj = AA_Fset[k - j] - AA_Fset[k - j - 1]; // delta F(k-j)
			for (int row = 0; row < nodeNum; row++) {
				for (int coord = 0; coord < 3; coord++) {
					D(row + nodeNum * coord, j) = deltaF_kj(row, coord); // make x,y,z-coordinate flat
					FkFlat(row + nodeNum * coord) = AA_Fset[k](row, coord);
				}

			}
		}

		Eigen::MatrixXd DT = D.transpose();
		Eigen::VectorXd theta = ((DT * D).inverse()) * (DT * FkFlat); // std::cout << theta << std::endl;

		AA_Qset[k + 1] = Q_LG;
		for (int j = 0; j < mk; j++) AA_Qset[k + 1] -= theta(j) * (AA_Gset[k - j] - AA_Gset[k - j - 1]);


		//printf(" AA time = %ld ms.\n", clock() - totaltime);
		//centerTop.row(k) = _shifttoInitialPos();
		//if (k >= 10) {
		//	double dis = (centerTop.row(k) - centerTop.row(k - 1)).norm();
		//	//std::cout << "iter" << iter << "terminal dis =" << dis*dis << std::endl;
		//	if (dis * dis < terminalValue) {
		//		free(SVDInputMatrix); free(SVDOutputMatrix);
		//		std::cout << " -- Converge before reach max iter time, iter Time = " << k << std::endl;
		//		Eigen::Vector3d topPos = centerTop.row(k);

		//		return; //return topPos;
		//	}
		//}

		printf(" iter time = %ld ms.\n", clock() - totaltime);
		return true;

	}


}

Eigen::Vector3d& soroPneumaticKinematics::AAsolverGradient(int maxIter) {

	for (int k = 1; k < maxIter; k++) {

		// -- local step
		long totaltime = clock();
		_localStepAA(AA_Qset[k], P_AA);
		//printf(" local step time = %ld ms.\n", clock() - totaltime);

		double energyIterInit = _calEnergyIterAA(AA_Qset[k], P_AA);

		if (energyIterInit > Eperv + eps) {

			// std::cout << "  iter - " << k << " perv energy is lower! perv is " << Eperv << "And QSet K gives" << energyIterInit <<  std::endl;
			AA_Qset[k] = Q_LG;
			_localStepAA(Q_LG, P_AA);

			double E_LG = _calEnergyIterAA(AA_Qset[k], P_AA);

			// after minimum number of iteration time we apply the terminal checking
			if (Eperv < E_LG && k > minIter && afterStopIterNum < maxVariationIterNum) {
				AAStopFlag = true;
				afterStopIterNum++;
				//printf("\n -- Terminal checking begin at iteration %d. \n\n", k);
				printf("\n -- Terminal checking happens once at iteration %d. \n\n", k);

				//nodePos = AA_Qset[k];
				//printf("\n---------------- Stop iteration at %d iter! ---------\n\n", k);
				//return _shifttoOriginalPosAA();
			}

			//else Eperv = _calEnergyIterAA(AA_Qset[k], P_AA);
			Eperv = E_LG;
		}
		else Eperv = energyIterInit;

		energyAAIter.push_back(Eperv);
		std::cout.precision(14); std::cout << "iter " << k << " energy = " << Eperv << std::endl;

		if (AAStopFlag && afterStopIterNum == maxVariationIterNum) {
			Eigen::VectorXd energyCompare(energyAAIter.size());
			for (int i = 0; i < energyAAIter.size(); i++)
				energyCompare(i) = energyAAIter[i];

			int minEnergyIter;
			energyCompare.minCoeff(&minEnergyIter);

			nodePos = AA_Qset[minEnergyIter];

			printf("\n --- Stop iteration at %d iter! The final solution is iter # %d.\n\n", k, minEnergyIter);
			return _shifttoOriginalPosAA();
		}

		/*if (AAStopFlag) {
			if (afterStopIterNum < maxVariationIterNum) afterStopIterNum++;
			else {

				Eigen::VectorXd energyCompare(energyAAIter.size());
				for (int i = 0; i < energyAAIter.size(); i++)
					energyCompare(i) = energyAAIter[i];

				int minEnergyIter;
				energyCompare.minCoeff(&minEnergyIter);

				nodePos = AA_Qset[minEnergyIter];

				printf("\n --- Stop iteration at %d iter! The final solution is iter # %d.\n\n", k, minEnergyIter);
				return _shifttoOriginalPosAA();
			}
		}*/


		//----------------------------
		// AA begin

		//totaltime = clock();

		_globalStepAA(P_AA, Q_LG);

		AA_Gset[k] = Q_LG;
		AA_Fset[k] = AA_Gset[k] - AA_Qset[k];

		//printf(" global step time = %ld ms.\n", clock() - totaltime);

		//totaltime = clock();

		int mk; if (k < m) mk = k; else mk = m;

		Eigen::MatrixXd D(3 * nodeNum, mk);	Eigen::VectorXd FkFlat(3 * nodeNum);
		for (int j = 0; j < mk; j++) {
			Eigen::MatrixXd deltaF_kj = AA_Fset[k - j] - AA_Fset[k - j - 1]; // delta F(k-j)
			for (int row = 0; row < nodeNum; row++) {
				for (int coord = 0; coord < 3; coord++) {
					D(row + nodeNum * coord, j) = deltaF_kj(row, coord); // make x,y,z-coordinate flat
					FkFlat(row + nodeNum * coord) = AA_Fset[k](row, coord);
				}

			}
		}

		Eigen::MatrixXd DT = D.transpose();
		Eigen::VectorXd theta = ((DT * D).inverse()) * (DT * FkFlat); // std::cout << theta << std::endl;

		AA_Qset[k + 1] = Q_LG;
		for (int j = 0; j < mk; j++) AA_Qset[k + 1] -= theta(j) * (AA_Gset[k - j] - AA_Gset[k - j - 1]);


		//printf(" AA time = %ld ms.\n", clock() - totaltime);
		//centerTop.row(k) = _shifttoInitialPos();
		//if (k >= 10) {
		//	double dis = (centerTop.row(k) - centerTop.row(k - 1)).norm();
		//	//std::cout << "iter" << iter << "terminal dis =" << dis*dis << std::endl;
		//	if (dis * dis < terminalValue) {
		//		free(SVDInputMatrix); free(SVDOutputMatrix);
		//		std::cout << " -- Converge before reach max iter time, iter Time = " << k << std::endl;
		//		Eigen::Vector3d topPos = centerTop.row(k);

		//		return; //return topPos;
		//	}
		//}

		//printf(" iter time = %ld ms.\n", clock() - totaltime);

	}
}

Eigen::Vector3d& soroPneumaticKinematics::AAsolverGradient_hyperMethod(int maxIter) {

	for (int k = 1; k < maxIter; k++) {

		// -- local step
		long totaltime = clock();
		_localStepAA(AA_Qset[k], P_AA);
		//printf(" local step time = %ld ms.\n", clock() - totaltime);

		double energyIterInit = _calEnergyIterAA(AA_Qset[k], P_AA);

		if (energyIterInit > Eperv + eps && !AAStopFlag) {

			std::cout << "  iter - " << k << " previous energy is lower as " << Eperv << "And QSet K gives" << energyIterInit << std::endl;
			AA_Qset[k] = Q_LG;
			_localStepAA(Q_LG, P_AA);

			double E_LG = _calEnergyIterAA(AA_Qset[k], P_AA);

			// after minimum number of iteration time we apply the terminal checking
			if (Eperv < E_LG && k > minIter) {
				AAStopFlag = true;
				printf("\n -- AA stop flag is activated at iteration %d. \n\n", k);
			}

			Eperv = E_LG;
		}
		else Eperv = energyIterInit;

		energyAAIter.push_back(Eperv);
		std::cout.precision(14); std::cout << "iter " << k << " energy = " << Eperv << std::endl;

		if (AAStopFlag) {
			afterStopIterNum++;
			if (afterStopIterNum == maxVariationIterNum) {
				nodePos = AA_Qset[k];
				printf("\n --- Stop iteration at %d iter!\n\n", k);
				return _shifttoOriginalPosAA();
			}

			_globalStepAA(P_AA, Q_LG);
			AA_Qset[k + 1] = Q_LG;
			continue;
		}

		//----------------------------
		// AA begin

		//totaltime = clock();

		_globalStepAA(P_AA, Q_LG);

		AA_Gset[k] = Q_LG;
		AA_Fset[k] = AA_Gset[k] - AA_Qset[k];

		//printf(" global step time = %ld ms.\n", clock() - totaltime);

		//totaltime = clock();

		int mk; if (k < m) mk = k; else mk = m;

		Eigen::MatrixXd D(3 * nodeNum, mk);	Eigen::VectorXd FkFlat(3 * nodeNum);
		for (int j = 0; j < mk; j++) {
			Eigen::MatrixXd deltaF_kj = AA_Fset[k - j] - AA_Fset[k - j - 1]; // delta F(k-j)
			for (int row = 0; row < nodeNum; row++) {
				for (int coord = 0; coord < 3; coord++) {
					D(row + nodeNum * coord, j) = deltaF_kj(row, coord); // make x,y,z-coordinate flat
					FkFlat(row + nodeNum * coord) = AA_Fset[k](row, coord);
				}

			}
		}

		Eigen::MatrixXd DT = D.transpose();
		Eigen::VectorXd theta = ((DT * D).inverse()) * (DT * FkFlat); // std::cout << theta << std::endl;

		AA_Qset[k + 1] = Q_LG;
		for (int j = 0; j < mk; j++) AA_Qset[k + 1] -= theta(j) * (AA_Gset[k - j] - AA_Gset[k - j - 1]);

	}
}

void soroPneumaticKinematics::saveCurrentPos() {
	FKPos = Eigen::MatrixXd::Zero(nodeNum, 3);
	FKPos = nodePos;
}

void soroPneumaticKinematics::clearAASystem(int iterTime) {
	Eperv = 99999999.9999;
	energyAAIter.clear();

	AA_Qset.resize(iterTime + 1);
	AA_Fset.resize(iterTime + 1);
	AA_Gset.resize(iterTime + 1);

	for (int i = 0; i < iterTime + 1; i++) {
		AA_Qset[i] = Eigen::MatrixXd::Zero(nodeNum, 3);
		AA_Fset[i] = Eigen::MatrixXd::Zero(nodeNum, 3);
		AA_Gset[i] = Eigen::MatrixXd::Zero(nodeNum, 3);
	}

	Q_LG = Eigen::MatrixXd::Zero(nodeNum, 3);
	iterQ = Eigen::MatrixXd::Zero(nodeNum, 3);
	P_AA.resize(eleNum);
	for (int i = 0; i < eleNum; i++) P_AA[i] = Eigen::MatrixXd::Zero(3, 4);

	//_updateQ(iterQ);	
	iterQ = nodePos;
	AA_Qset[0] = iterQ; //Q0

	_localStepAA(AA_Qset[0], P_AA);
	double initEnergyAA = _calEnergyIterAA(AA_Qset[0], P_AA);
	cout.precision(17); std::cout << "iter " << 0 << " energy = " << initEnergyAA << std::endl;
	energyAAIter.push_back(initEnergyAA);

	_globalStepAA(P_AA, AA_Qset[1]); //run first iteration to compute Q1

	AA_Gset[0] = AA_Qset[1];
	AA_Fset[0] = AA_Gset[0] - AA_Qset[0]; // F0
}

void soroPneumaticKinematics::solve(int iterTime) {

	//Eigen::MatrixXd centerTop(iterTime, 3); centerTop = Eigen::MatrixXd::Zero(iterTime, 3);

	for (int iter = 0; iter < iterTime; iter++) {

		//std::cout << "begin iter " << iter;
		if (iter % 1 == 1) std::cout << " ---- now running iter. " << iter << std::endl;

		// -- compute current matrix and fill matrix B

		long time = clock();

#pragma omp parallel
		{
#pragma omp for
			for (int eleIndex = 0; eleIndex < model->GetTetraNumber(); eleIndex++) {

				Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
				for (int j = 0; j < 4; j++) P.col(j) = nodePos.row(tetTable(eleIndex, j));

				Eigen::Matrix3d T = (InverseP[eleIndex] * ((P * N).transpose())).transpose();

				for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
					SVDInputMatrix[9 * eleIndex + 3 * i + j] = T(i, j);

				JacobiSVD<Eigen::MatrixXd> svd(T, ComputeThinU | ComputeThinV);
				Matrix3d V = svd.matrixV(), U = svd.matrixU();

				LocalCoord[eleIndex] = U * V.transpose() * LocalGoal[eleIndex];

				Eigen::Vector3d centerLocalCoord = LocalCoord[eleIndex].rowwise().mean();

				for (int j = 0; j < 4; j++) for (int k = 0; k < 3; k++)
					LocalCoord[k](4 * eleIndex + j) = (LocalCoord[eleIndex](k, j) - centerLocalCoord(k)) * weight(eleIndex);
			}
		}

		/*CompSVD_CUDA(SVDInputMatrix, SVDOutputMatrix, model->GetTetraNumber());

#pragma omp parallel
		{
#pragma omp for
			for (int eleIndex = 0; eleIndex < model->GetTetraNumber(); eleIndex++) {

				Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
				for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
					R(i, j) = SVDOutputMatrix[9 * eleIndex + 3 * i + j];
				LocalCoord[eleIndex] = R * LocalGoal[eleIndex];
			}
		}		*/
		//printf(" SVD takes %ld ms.\n", clock() - time);

		double energyIter = _calEnergyIterAA(nodePos, LocalCoord);
		//double energyIter = calEnergyIter();
		cout.precision(17); std::cout << "iter " << iter << " energy = " << energyIter << std::endl;
		energyAAIter.push_back(energyIter);

#pragma omp parallel
		{
#pragma omp for
			for (int eleIndex = 0; eleIndex < model->GetTetraNumber(); eleIndex++) {
				Eigen::Vector3d centerLocalCoord = LocalCoord[eleIndex].rowwise().mean();
				for (int j = 0; j < 4; j++) for (int k = 0; k < 3; k++)
					VectorBSide[k](4 * eleIndex + j) = (LocalCoord[eleIndex](k, j) - centerLocalCoord(k)) * weight(eleIndex);
			}
		}

		std::vector<Eigen::VectorXd> ATB(3);
		// -- solve linear system
		time = clock();
#pragma omp parallel
		{
#pragma omp for
			for (int i = 0; i < 3; i++) {
				ATB[i] = matAT * VectorBSide[i];
				VectorXPosition[i] = Solver.solve(ATB[i]);
			}
		}
		//printf(" solve Linear equation %ld ms.\n", clock() - time);

		// -- update mesh position
#pragma omp parallel
		{
#pragma omp for
			for (int i = 0; i < model->GetNodeNumber(); i++) {
				for (int j = 0; j < 3; j++)
					nodePos(i, j) = VectorXPosition[j](i);
			}
		}

		centerTop.row(iter) = _shifttoOriginalPosAA();

		// for (int i = 0; i < 20; i++) std::cout << " Node Pos = " << nodePos(i, 0) << " , " << nodePos(i, 1) << " , " << nodePos(i, 2) << std::endl;

		time = clock();
		// shift back node position to initial orientation / Check terminal condition

		//std::cout << "Center top Pos = " << centerTop(iter, 0) << " , " << centerTop(iter, 1) << " , " << centerTop(iter, 2) << std::endl;

		if (iter >= 10) {
			double dis = (centerTop.row(iter) - centerTop.row(iter - 1)).norm();
			//std::cout << "iter" << iter << "terminal dis =" << dis*dis << std::endl;
			if (dis * dis < terminalValue) {
				free(SVDInputMatrix); free(SVDOutputMatrix);
				std::cout << " -- Converge before reach max iter time, iter Time = " << iter << std::endl;
				Eigen::Vector3d topPos = centerTop.row(iter);

				return; //return topPos;
			}
		}

		// printf(" check terminal takes %ld ms.\n", clock() - time);

	}
	Eigen::Vector3d finalTopPos = centerTop.row(iterTime - 1);
	free(SVDInputMatrix); free(SVDOutputMatrix);
	std::cout << " -- Reach max iter time, iter Time = " << iterTime << std::endl;

	this->_updateMeshPos();

	return;
	//return finalTopPos;


}

void soroPneumaticKinematics::AAsolver(int iter, bool timer) {
	if (timer) AAsolverIter(iter); // iter -> current iter time
	else {
		for (int i = 0; i < iter; i++) {
			if (AAsolverIter(i) == false) break; // iter -> total iter time
		}
	}
}

bool soroPneumaticKinematics::AAsolverIter(int iterTime) {

	int k = iterTime;

	long totaltime = clock();
	_localStepAA(AA_Qset[k], P_AA);	// local step

	//printf(" local step time = %ld ms.\n", clock() - totaltime);
	double energyIterInit = _calEnergyIterAA(AA_Qset[k], P_AA);

	if (energyIterInit >= Eperv) {
		//std::cout << "      iter - " << k << " perv energy is lower!" << std::endl;
		AA_Qset[k] = Q_LG; _localStepAA(Q_LG, P_AA);
		Eperv = _calEnergyIterAA(AA_Qset[k], P_AA);
	}
	else Eperv = energyIterInit;
	cout.precision(17); std::cout << "iter " << k << " energy = " << Eperv << std::endl;
	energyAAIter.push_back(Eperv);

	//_shifttoInitialPos(); // update the mesh based on last iteration

	// determine if the iterlation need to continue;
	nodePos = AA_Qset[k];
	centerTop.row(k) = _shifttoOriginalPosAA();
	if (k >= 100 && pow(((centerTop.row(k) - centerTop.row(k - 1)).norm()), 2.0) < terminalValue) {
		std::cout << " -- Converge! iter Time = " << k << std::endl; return false;
	}

	//----------------------------
	// AA begin

	//totaltime = clock();

	_globalStepAA(P_AA, Q_LG);
	AA_Gset[k] = Q_LG; AA_Fset[k] = AA_Gset[k] - AA_Qset[k];

	//printf(" global step time = %ld ms.\n", clock() - totaltime);

	//totaltime = clock();

	int mk; if (k < m) mk = k; else mk = m;

	Eigen::MatrixXd D(3 * nodeNum, mk);	Eigen::VectorXd FkFlat(3 * nodeNum);
	for (int j = 0; j < mk; j++) {
		Eigen::MatrixXd deltaF_kj = AA_Fset[k - j] - AA_Fset[k - j - 1]; // delta F(k-j)
		for (int row = 0; row < nodeNum; row++) {
			for (int coord = 0; coord < 3; coord++) {
				D(row + nodeNum * coord, j) = deltaF_kj(row, coord); // make x,y,z-coordinate flat
				FkFlat(row + nodeNum * coord) = AA_Fset[k](row, coord);
			}

		}
	}

	//for (int row = 0; row < nodeNum; row++) {
	//	FkFlat(row) = AA_Fset[k](row, 0); FkFlat(row + nodeNum) = AA_Fset[k](row, 1); FkFlat(row + 2 * nodeNum) = AA_Fset[k](row, 2);
	//}
	Eigen::MatrixXd DT = D.transpose();
	Eigen::VectorXd theta = ((DT * D).inverse()) * (DT * FkFlat); // std::cout << theta << std::endl;

	AA_Qset[k + 1] = Q_LG;
	for (int j = 0; j < mk; j++) AA_Qset[k + 1] -= theta(j) * (AA_Gset[k - j] - AA_Gset[k - j - 1]);


	//printf(" AA time = %ld ms.\n", clock() - totaltime);
	//centerTop.row(k) = _shifttoInitialPos();
	//if (k >= 10) {
	//	double dis = (centerTop.row(k) - centerTop.row(k - 1)).norm();
	//	//std::cout << "iter" << iter << "terminal dis =" << dis*dis << std::endl;
	//	if (dis * dis < terminalValue) {
	//		free(SVDInputMatrix); free(SVDOutputMatrix);
	//		std::cout << " -- Converge before reach max iter time, iter Time = " << k << std::endl;
	//		Eigen::Vector3d topPos = centerTop.row(k);

	//		return; //return topPos;
	//	}
	//}

	printf(" iter time = %ld ms.\n", clock() - totaltime);
	return true;


}

Eigen::Vector3d& soroPneumaticKinematics::_shifttoOriginalPosAA() {

	int selectNum = shiftInit.rows();

	Eigen::MatrixXd shiftCurrent = Eigen::MatrixXd::Constant(selectNum, 3, 1.0);
	Eigen::Vector3d shiftCenterCurrent = Eigen::Vector3d::Zero();

	Eigen::MatrixXd shiftOrig = Eigen::MatrixXd::Constant(selectNum, 3, 1.0);
	Eigen::Vector3d shiftCenterOrig = Eigen::Vector3d::Zero();

	int selectIndex = 0;
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (shiftTable(i, 0) == 1) {
			shiftCurrent.row(selectIndex) = nodePos.row(i);
			for (int i = 0; i < 3; i++) shiftCenterCurrent[i] += shiftCurrent(selectIndex, i) / selectNum;

			shiftOrig.row(selectIndex) = nodePosInit.row(i);
			for (int i = 0; i < 3; i++) shiftCenterOrig[i] += shiftOrig(selectIndex, i) / selectNum;

			selectIndex++;
		}
	}
	for (int i = 0; i < selectIndex; i++) {
		shiftCurrent.row(i) -= shiftCenterCurrent;
		shiftOrig.row(i) -= shiftCenterOrig;
	}
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(shiftOrig.transpose() * shiftCurrent, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d rotateMat = svd.matrixU() * (svd.matrixV().transpose());

	Eigen::Vector3d t = -rotateMat * shiftCenterCurrent + shiftCenterOrig;

#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < model->GetNodeNumber(); i++) {
			Eigen::Vector3d rotateNode = nodePos.row(i);
			nodePos.row(i) = rotateMat * rotateNode + t;
		}
	}

	_updateMeshPos();

	// compute top center and send back
	Eigen::Vector3d topCenter = Eigen::Vector3d::Zero(); int topSelectNum = 0;
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (shiftTable(i, 1) == 1) { topCenter += nodePos.row(i); topSelectNum++; }
	}
	topCenter = topCenter / topSelectNum;
	return topCenter;
}

// output of this function: the center of model tip
Eigen::Vector3d& soroPneumaticKinematics::_shifttoInitialPos() {

	/*Least - Squares Rigid Motion Using SVD*/


	//----------------------------------
	// build system and move to center pos

	int selectNum = shiftInit.rows();

	Eigen::MatrixXd shiftCurrent = Eigen::MatrixXd::Constant(selectNum, 3, 1.0);
	Eigen::Vector3d shiftPosCurrent = Eigen::Vector3d::Zero();

	int selectIndex = 0;
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (shiftTable(i, 0) == 1) {
			shiftCurrent.row(selectIndex) = nodePos.row(i);
			for (int i = 0; i < 3; i++) shiftPosCurrent[i] += shiftCurrent(selectIndex, i) / selectNum;
			selectIndex++;
		}
	}
	for (int i = 0; i < model->GetNodeNumber(); i++)
		nodePos.row(i) += (shiftPosInit - shiftPosCurrent);


	//----------------------------------
	// rotate the select region to fix initial pos
		//find affine transformation matrix -------- SVD[ (CMat)-1 * InitMat ]

	selectIndex = 0;
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (shiftTable(i, 0) == 1) {
			shiftCurrent.row(selectIndex) = nodePos.row(i); selectIndex++;
		}
	}

	Eigen::MatrixXd InverseCurrent = (shiftCurrent.transpose()).completeOrthogonalDecomposition().pseudoInverse();
	Eigen::MatrixXd TransformMat = ((InverseCurrent.transpose()) * shiftInit).transpose();

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(TransformMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d rotateMat = svd.matrixU() * (svd.matrixV().transpose());

	std::cout << "first rotateMat = " << rotateMat << std::endl;

#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < model->GetNodeNumber(); i++) {
			Eigen::Vector3d rotateNode = nodePos.row(i);
			nodePos.row(i) = rotateMat * rotateNode;
		}
	}

	selectIndex = 0;
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (shiftTable(i, 0) == 1) {
			shiftCurrent.row(selectIndex) = nodePos.row(i); selectIndex++;
		}
	}

	for (int i = 0; i < 20; i++)
		std::cout << " Node Pos (shiftInit) = "
		<< shiftInit(i, 0) << " , " << shiftInit(i, 1) << " , " << shiftInit(i, 2) << std::endl;

	for (int i = 0; i < 20; i++)
		std::cout << " Node Pos (shiftCurrent) = "
		<< shiftCurrent(i, 0) << " , " << shiftCurrent(i, 1) << " , " << shiftCurrent(i, 2) << std::endl;




	//shift to origin center
	Eigen::Vector3d bottomCenter = Eigen::Vector3d::Zero(); int bottomSelectNum = 0;
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (shiftTable(i, 2) == 1) {
			bottomCenter += nodePos.row(i); bottomSelectNum++;
		}
	}

	for (int i = 0; i < model->GetNodeNumber(); i++)
		nodePos.row(i) -= bottomCenter / bottomSelectNum;

	_updateMeshPos();

	// for (int i = 0; i < 20; i++)  std::cout << " Node Pos (after final shifting) = " << nodePos(i, 0) << " , " << nodePos(i, 1) << " , " << nodePos(i, 2) << std::endl;


	// compute top center and send back
	Eigen::Vector3d topCenter = Eigen::Vector3d::Zero(); int topSelectNum = 0;
	for (int i = 0; i < model->GetNodeNumber(); i++) {
		if (shiftTable(i, 1) == 1) { topCenter += nodePos.row(i); topSelectNum++; }
	}
	topCenter = topCenter / topSelectNum;
	return topCenter;

}

void soroPneumaticKinematics::_updateMeshPos() {
	for (GLKPOSITION Pos = model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)model->GetNodeList().GetNext(Pos);
		Eigen::Vector3d pos = nodePos.row(node->GetIndexNo());
		node->SetCoord3D(pos);
	}
}

void soroPneumaticKinematics::_localStepAA(Eigen::MatrixXd& inputQ, std::vector<Eigen::MatrixXd>& P_AA) {

#pragma omp parallel
	{
#pragma omp for
		for (int eleIndex = 0; eleIndex < model->GetTetraNumber(); eleIndex++) {

			Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
			for (int j = 0; j < 4; j++) P.col(j) = inputQ.row(tetTable(eleIndex, j));

			Eigen::Matrix3d T = (InverseP[eleIndex] * ((P * N).transpose())).transpose();

			/*for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
				SVDInputMatrix[9 * eleIndex + 3 * i + j] = T(i, j);*/


			JacobiSVD<Eigen::MatrixXd> svd(T, ComputeThinU | ComputeThinV);
			Matrix3d V = svd.matrixV(), U = svd.matrixU();
			P_AA[eleIndex] = U * V.transpose() * LocalGoal[eleIndex];


		}
	}

	/*CompSVD_CUDA(SVDInputMatrix, SVDOutputMatrix, model->GetTetraNumber());

#pragma omp parallel
	{
#pragma omp for
		for (int eleIndex = 0; eleIndex < model->GetTetraNumber(); eleIndex++) {

			Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
			for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
				R(i, j) = SVDOutputMatrix[9 * eleIndex + 3 * i + j];
			P_AA[eleIndex] = R * LocalGoal[eleIndex];
		}
	}*/

}

void soroPneumaticKinematics::_globalStepAA(std::vector<Eigen::MatrixXd>& P_AA, Eigen::MatrixXd& outputQ) {


#pragma omp parallel
	{
#pragma omp for
		for (int eleIndex = 0; eleIndex < model->GetTetraNumber(); eleIndex++) {
			Eigen::Vector3d centerLocalCoord = P_AA[eleIndex].rowwise().mean();
			for (int j = 0; j < 4; j++) for (int k = 0; k < 3; k++)
				VectorBSide[k](4 * eleIndex + j) =
				(P_AA[eleIndex](k, j) - centerLocalCoord(k)) * weight(eleIndex);
		}
	}

	// -- solve linear system

#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < 3; i++) {
			Eigen::VectorXd ATB = matAT * VectorBSide[i];
			VectorXPosition[i] = Solver.solve(ATB);
		}
	}

	// -- update mesh position and iterQ
#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < model->GetNodeNumber(); i++) {
			//for (int j = 0; j < 3; j++) nodePos(i, j) = VectorXPosition[j](i);
			for (int j = 0; j < 3; j++) outputQ(i, j) = VectorXPosition[j](i);

		}
	}
	//outputQ = nodePos;
	//_shifttoInitialPos();

}

//void soroDeformAA::_updateQ(Eigen::MatrixXd& iterQ) {
//	// -- update mesh position and iterQ
//#pragma omp parallel
//	{
//#pragma omp for
//		for (int i = 0; i < model->GetNodeNumber(); i++) {
//			for (int j = 0; j < 3; j++) nodePos(i, j) = VectorXPosition[j](i);
//			outputQ = nodePos;
//		}
//	}
//}


double soroPneumaticKinematics::calEnergyIter() {

	// the energy = \sum (N_i V_i - P_i(N_i V_i))^2

	double energy = 0.0;
	Eigen::VectorXd energyIter = Eigen::VectorXd::Zero(model->GetTetraNumber());

#pragma omp parallel
	{
#pragma omp for  
		for (int omptime = 0; omptime < OMPCORE; omptime++) {
			for (GLKPOSITION Pos = model->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* Tetra = (QMeshTetra*)model->GetTetraList().GetNext(Pos);
				if (Tetra->GetIndexNo() < omptime * EachCore) continue;
				else if (Tetra->GetIndexNo() > (1 + omptime) * EachCore) break;
				int fdx = Tetra->GetIndexNo();

				double tetWeight = 1.0;
				if (Tetra->isRigid) tetWeight = weightHard;
				else if (Tetra->chamberElement) tetWeight = weightChamber;

				Eigen::MatrixXd currV = Eigen::MatrixXd::Zero(VELE, 3); QMeshNode* nodes[VELE];
				for (int i = 0; i < VELE; i++) {
					nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
					nodes[i]->GetCoord3D(currV(i, 0), currV(i, 1), currV(i, 2));
				}
				currV = N * currV;

				Matrix3d transMat = (InverseP[fdx] * currV).transpose();
				JacobiSVD<Eigen::MatrixXd> svd(transMat, ComputeThinU | ComputeThinV);
				Matrix3d R = svd.matrixU() * (svd.matrixV().transpose());

				Eigen::MatrixXd eleE = currV - (R * LocalGoal[fdx]).transpose();

				energyIter(Tetra->GetIndexNo()) += eleE.squaredNorm() * sqrt(tetWeight);

			}
		}
	}
	/*cout.precision(17);
	std::cout << energyIter.sum() << std::endl;*/

	return energyIter.sum();

}

double soroPneumaticKinematics::_calEnergyIterAA(Eigen::MatrixXd& inputQ, std::vector<Eigen::MatrixXd>& P_AA) {

	// the energy = \sum (N_i V_i - P_i(N_i V_i))^2

	long calEnergyTime = clock();
	Eigen::VectorXd energyIter = Eigen::VectorXd::Zero(model->GetTetraNumber());

#pragma omp parallel
	{
#pragma omp for
		for (int eleIndex = 0; eleIndex < model->GetTetraNumber(); eleIndex++) {

			Eigen::MatrixXd currV = Eigen::MatrixXd::Zero(4, 3);
			for (int j = 0; j < 4; j++) currV.row(j) = inputQ.row(tetTable(eleIndex, j));
			Eigen::MatrixXd eleE = N * currV - P_AA[eleIndex].transpose();
			//energyIter(eleIndex) += eleE.squaredNorm() * weight(eleIndex) * weight(eleIndex);
			energyIter(eleIndex) += eleE.squaredNorm() * weight(eleIndex);


		}
	}

	// printf(" calculate energy AA takes = %ld ms.\n", clock() - calEnergyTime);
	return energyIter.sum();

}

void soroPneumaticKinematics::updateExpandRatio(int iterTime) {

	// update expand ratio and target goal
	std::cout << "this iteration expanding ratio = "
		<< expandRatio[0] << ", " << expandRatio[1] << ", " << expandRatio[2] << std::endl;

	for (GLKPOSITION Pos = model->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* tetra = (QMeshTetra*)model->GetTetraList().GetNext(Pos);
		int tetindex = tetra->GetIndexNo();
		for (int i = 0; i < 4; i++)
			tetTable(tetindex, i) = tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		for (int chamberIndex = 0; chamberIndex < 3; chamberIndex++) {
			if (tetra->isChamber[chamberIndex] == true) {
				tetSelected(tetindex) = expandRatio[chamberIndex]; break;
			}
		}
	}
	_compLocalGoalandInverse();

	// initialize AA system
	for (int i = 0; i < energyAAIter.size(); i++) energyAAIter[i] = 0;
	buildSystemAA(iterTime, true);
}

void soroPneumaticKinematics::updateExpandRatio(Eigen::Vector3d& actuationPara) {

	// update expand ratio and target goal
	std::cout << "this iteration expanding ratio = "
		<< actuationPara[0] << ", " << actuationPara[1] << ", " << actuationPara[2] << std::endl;

	for (GLKPOSITION Pos = model->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* tetra = (QMeshTetra*)model->GetTetraList().GetNext(Pos);
		int tetindex = tetra->GetIndexNo();
		for (int i = 0; i < 4; i++)
			tetTable(tetindex, i) = tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		for (int chamberIndex = 0; chamberIndex < 3; chamberIndex++) {
			if (tetra->isChamber[chamberIndex] == true) {
				tetSelected(tetindex) = actuationPara[chamberIndex]; break;
			}
		}
	}
	_compLocalGoalandInverse();

	// initialize AA system
	for (int i = 0; i < energyAAIter.size(); i++) energyAAIter[i] = 0;
}

void soroPneumaticKinematics::outputEnergyCurveAA(Eigen::VectorXd& energyOperator) {

	energyOperator.resize(energyAAIter.size());
	for (int i = 0; i < energyAAIter.size(); i++) {
		energyOperator(i) = energyAAIter[i];
	}
	double maxE = energyOperator.maxCoeff();
	double minE = energyOperator.minCoeff();

	for (int i = 0; i < energyAAIter.size(); i++) {
		energyOperator(i) = log10((energyOperator(i) - minE) / (maxE - minE));
	}
	//std::cout << "-------------------------------" << std::endl << energyOperator << std::endl;

}


void soroPneumaticKinematics::_compLocalGoalandInverse() {

#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < model->GetTetraNumber(); i++) {

			Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
			for (int j = 0; j < 4; j++) P.col(j) = nodePosInit.row(tetTable(i, j));
			Eigen::Vector3d center = P.rowwise().mean();

			if (tetSelected(i) > 0.00001)
				for (int j = 0; j < 4; j++) P.col(j) = (P.col(j) - center) * pow(tetSelected(i), 1.0 / 3);
			else for (int j = 0; j < 4; j++) P.col(j) = P.col(j) - center;

			LocalGoal[i] = P;
			InverseP[i] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();

			if (InverseP[i].mean() - InverseP[i].mean() != 0) cout << InverseP[i] << " pseudo inverse ERROR!" << std::endl;

		}
	}

}

void soroPneumaticKinematics::_compLocalGoalandInverse_grasping() {

	for (GLKPOSITION Pos = model->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)model->GetTetraList().GetNext(Pos);
		int fdx = Tet->GetIndexNo();
		Tet->selected = false;
		for (int i = 0; i < VELE; i++) {
			QMeshNode* Tet_Node = Tet->GetNodeRecordPtr(i + 1);
			if (Tet_Node->selected == true) {
				Tet->selected = true;
				break;
			}
		}

		QMeshNode* nodes[VELE];
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, VELE);
		double center[3] = { 0 };

		for (int i = 0; i < VELE; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= VELE;
		for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];


		for (int i = 0; i < VELE; i++) {
			QMeshFace* Face = Tet->GetFaceRecordPtr(i + 1);
			if (Face->isChamberFace == true) {
				//cout << "Tet index = " << Tet->GetIndexNo() << endl;
				int NodeIndex[3];
				for (int j = 0; j < 3; j++) {
					QMeshNode* NodeFace = Face->GetNodeRecordPtr(j);
					for (int k = 0; k < 4; k++) {
						QMeshNode* NodeTet = Tet->GetNodeRecordPtr(k + 1);
						if (NodeFace == NodeTet) { NodeIndex[j] = k; break; }
					}
				}
				double facecenter[3] = { 0 };
				for (int j = 0; j < 3; j++) {
					facecenter[j] = (P(j, NodeIndex[0]) + P(j, NodeIndex[1]) + P(j, NodeIndex[2])) / 3;
					for (int k = 0; k < 3; k++) {
						P(j, NodeIndex[k]) = P(j, NodeIndex[k]) * fingerPressure
							+ facecenter[j] * (1 - fingerPressure);
						//P(j, NodeIndex[k]) = P(j, NodeIndex[k]) * pow(ExpandRatio[chamberindex], 1.0 / 2)
						//	+ facecenter[j] * (1 - pow(ExpandRatio[chamberindex], 1.0 / 2));
					}
				}
				break;
			}
		}


		//This is for selected region size
		for (int chamberIndex = 0; chamberIndex < 3; chamberIndex++) {
			if (Tet->isChamber[chamberIndex] == true) {
				for (int i = 0; i < VELE; i++) {
					for (int j = 0; j < 3; j++) { P(j, i) *= pow(expandRatio[chamberIndex], 1.0 / 3); }
				}
				break;
			}
		}

		//solving pseudoInverse with GLKMATRIX Lib, Eigen may occur error here
		LocalGoal[fdx] = P;

		GLKMatrix GLKP(3, VELE), GLKInverseP(3, VELE);
		InverseP[fdx] = Eigen::MatrixXd::Zero(3, 4);

		for (int i = 0; i < 3; i++) { for (int j = 0; j < 4; j++)  GLKP(i, j) = P(i, j); }

		GLKMatrix TP(VELE, 3), GLKATA(3, 3);
		GLKMatrixLib::Transpose(GLKP, 3, VELE, TP);
		GLKMatrixLib::Mul(GLKP, TP, 3, VELE, 3, GLKATA);

		if (!GLKMatrixLib::Inverse(GLKATA, 3)) {
			printf("ERROR in finding Inverse!\n");
			getchar();
		}
		GLKMatrixLib::Mul(GLKATA, GLKP, 3, 3, VELE, GLKInverseP);

		for (int i = 0; i < 3; i++) { for (int j = 0; j < 4; j++) InverseP[fdx](i, j) = GLKInverseP(i, j); }

	}

}

void soroPneumaticKinematics::_buildMatA() {

	double c1 = -1.0 / 4.0, c2 = 1 + c1;

	int AMatRowNum = VELE * model->GetTetraNumber();
	Eigen::SparseMatrix<double> matA(AMatRowNum, model->GetNodeNumber());
	matA.reserve(VectorXi::Constant(AMatRowNum, 1000));

	for (int eleIndex = 0; eleIndex < model->GetTetraNumber(); eleIndex++) {
		for (int AMatRow = 0; AMatRow < 4; AMatRow++) {
			for (int AMatCol = 0; AMatCol < 4; AMatCol++) {
				int AMatRowSum = 4 * eleIndex + AMatRow;
				if (AMatRow == AMatCol) matA.insert(AMatRowSum, tetTable(eleIndex, AMatCol)) = c2 * weight(eleIndex);
				else matA.insert(AMatRowSum, tetTable(eleIndex, AMatCol)) = c1 * weight(eleIndex);
			}
		}
	}

	matA.makeCompressed();

	matAT = matA.transpose(); matATA = matAT * matA;
	Solver.compute(matATA);

}

void soroPneumaticKinematics::inputTrajectoryPara() {

}