#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "soroPneumaticKinematics.h"
#include <iostream>
#include <fstream>

void MainWindow::createActions_SoftRobotFunction() {

	/* pre-process sytem, input model */
	connect(ui->pushButton_soroPreprocess, SIGNAL(released()), this, SLOT(soroPreProcessSystem()));

	/* run forward kinematics based on input actuator */
	connect(ui->pushButton_runFK, SIGNAL(released()), this, SLOT(soroForwardKinmeatics()));

	/* run inverse kinematics with path following task */
	connect(ui->pushButton_runIK, SIGNAL(released()), this, SLOT(soroInverseKinmeatics()));

}

/*--------------------------------------------*/
// Pneumatic driven soft manipulator function
/*--------------------------------------------*/

soroPneumaticKinematics* soroPneumaticKinematicsOperator;
void MainWindow::soroPreProcessSystem() {

	int maxiterTime = 1000;

	PolygenMesh* threeChamberMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch* model = (QMeshPatch*)threeChamberMesh->GetMeshList().GetHead();

	this->_shirftModeltoXYPlane(model);

	this->_inputChamberSelection(model);

	soroPneumaticKinematicsOperator = new soroPneumaticKinematics(model, maxiterTime);
	soroPneumaticKinematicsOperator->buildSimulationSystem();

	

	pGLK->refresh(true);

}

void MainWindow::soroForwardKinmeatics() {

	int maxitertime = 1000;
	Eigen::Vector3d tipPosComp;
	tipPosComp << ui->doubleSpinBox_A1->value(), ui->doubleSpinBox_A2->value(), ui->doubleSpinBox_A3->value();

	for (int chamberNum = 0; chamberNum < 3; chamberNum++)
		soroPneumaticKinematicsOperator->expandRatio[chamberNum] = tipPosComp(chamberNum);

	soroPneumaticKinematicsOperator->updateExpandRatio(maxitertime);

	//soroPneumaticKinematicsOperator->initSolverSingle(maxitertime, true);

	// hyber solver with Aderson Acceleration.
	soroPneumaticKinematicsOperator->hyberSolverSingle(maxitertime, false);

	pGLK->refresh(true);
	
}

void MainWindow::soroInverseKinmeatics() {
	soroPneumaticKinematicsOperator->computeInverseKinematics();
	pGLK->refresh(true);
}


void MainWindow::_inputChamberSelection(QMeshPatch* soroModel) {

	/*----- Chamber selection -----*/

	char chamber_selection[256];
	strcpy(chamber_selection, "../data/soro_chamber.txt");
	//strcpy(chamber_selection, "soro_chamber.txt");

	//strcpy(chamber_selection, "D:\\SORO_simulation\\Model\\soro_chamber.txt");


	std::ifstream chamberFile(chamber_selection);
	if (!chamberFile)
		std::cerr << "Sorry!We were unable to open the file!\n";

	Eigen::MatrixXi chamberIndex = Eigen::MatrixXi::Zero(soroModel->GetTetraNumber(), 5);

	//string line;
	int lineIndex = 0;
	std::string sss;
	while (std::getline(chamberFile, sss))
	{
		const char* c = sss.c_str();
		sscanf(c, "%d:%d:%d:%d:%d:",
			&chamberIndex(lineIndex, 0), &chamberIndex(lineIndex, 1), &chamberIndex(lineIndex, 2), &chamberIndex(lineIndex, 3), &chamberIndex(lineIndex, 4));
		lineIndex++;
	}

	chamberFile.close();

	//std::cout << chamberIndex << std::endl;

	for (GLKPOSITION Pos = soroModel->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* element = (QMeshTetra*)soroModel->GetTetraList().GetNext(Pos);
		//element index start from 1
		for (int i = 0; i < 3; i++) {
			if (chamberIndex(element->GetIndexNo() - 1, i + 1) == 1) {
				element->isChamber[i] = true; break;
			}
		}
	}

	std::cout << "finish input the chamber selection!" << std::endl;

	/*----- Rigid and handle node selection -----*/

	char rigid_selection[256];
	strcpy(rigid_selection, "../data/soro_rigid_handle.txt");
	//strcpy(rigid_selection, "soro_rigid_handle.txt");

	//strcpy(rigid_selection, "D:\\SORO_simulation\\Model\\soro_rigid_handle.txt");

	std::ifstream rigidRegion(rigid_selection);
	if (!rigidRegion)
		std::cerr << "Sorry!We were unable to open the file!\n";

	Eigen::MatrixXi rigidIndex = Eigen::MatrixXi::Zero(soroModel->GetTetraNumber(), 3);

	//string line;
	lineIndex = 0;
	while (getline(rigidRegion, sss))
	{
		const char* c = sss.c_str();
		sscanf(c, "%d:%d:%d:",
			&rigidIndex(lineIndex, 0), &rigidIndex(lineIndex, 1), &rigidIndex(lineIndex, 2));
		lineIndex++;
	}

	rigidRegion.close();

	for (GLKPOSITION Pos = soroModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)soroModel->GetNodeList().GetNext(Pos);
		//node index start from 1
		if (rigidIndex(node->GetIndexNo() - 1, 2) == 1) node->selectedRigid = true;
		if (rigidIndex(node->GetIndexNo() - 1, 1) == 1) node->shirftSelect = true;
	}

	for (GLKPOSITION Pos = soroModel->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* element = (QMeshTetra*)soroModel->GetTetraList().GetNext(Pos);
		for (int i = 0; i < 4; i++) {
			if (element->GetNodeRecordPtr(i + 1)->selectedRigid == true) {
				element->isRigid = true;
				for (int j = 1; j < 5; j++) element->GetFaceRecordPtr(j)->isHardDraw = true;
				break;
			}
		}
	}

	std::cout << "finish input the weighting selection!" << std::endl;
}

void MainWindow::_shirftModeltoXYPlane(QMeshPatch* soroModel) {

	double minY = 999990.99; double pp[3]; double center[3] = { 0 };
	for (GLKPOSITION Pos = soroModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)soroModel->GetNodeList().GetNext(Pos);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		for (int i = 0; i < 3; i++) center[i] += pp[i];
		if (pp[1] < minY) minY = pp[1];
	}
	for (int i = 0; i < 3; i++) center[i] /= soroModel->GetNodeNumber();
	for (GLKPOSITION Pos = soroModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)soroModel->GetNodeList().GetNext(Pos);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		pp[1] -= minY; pp[0] -= center[0]; pp[2] -= center[2];
		node->SetCoord3D(pp[0], pp[1], pp[2]);
	}

}