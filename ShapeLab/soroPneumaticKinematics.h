#pragma once
#include "..\QMeshLib\PolygenMesh.h"
#include "..\QMeshLib\QMeshPatch.h"
#include "..\QMeshLib\QMeshNode.h"
#include "..\QMeshLib\QMeshFace.h"

#include "..\QMeshLib\QMeshTetra.h"

#include "..\GLKLib\GLKObList.h"
#include "..\GLKLib\GLKGeometry.h"
#include "..\GLKLib\GLKMatrixLib.h"

// System includes
#include <stdio.h>
#include <assert.h>

class soroPneumaticKinematics
{

public:
    soroPneumaticKinematics(QMeshPatch* inputMesh, int maxIterTime);
    ~soroPneumaticKinematics() { }

    void buildSimulationSystem();

    bool solver(int iterTime, bool accelerate);
    void solve(int iterTime);

    void AAsolver(int iter, bool timer);

    bool AAsolverIter(int iterTime);

    Eigen::Vector3d& AAsolverGradient(int maxIter);
    Eigen::Vector3d& AAsolverGradient_hyperMethod(int maxIter);


    void clearAASystem(int iterTime);

    double expandRatio[3] = { 2.8,2.1,3.15 };

    void updateExpandRatio(int iterTime);
    void updateExpandRatio(Eigen::Vector3d& actuationPara);

    void outputEnergyCurveAA(Eigen::VectorXd& energyOperator);

    void saveCurrentPos();
    Eigen::Vector3d& getTipPos();

    /* 12-Sept-2020 final solution*/
    bool initSolver(int iterTime, bool checkTerminal, bool gradientFK);
    bool andersonSolver(int iterTime);
    Eigen::Vector3d& hyberSolverSingle(int maxiterTime, bool gradientFK);
    Eigen::Vector3d& initSolverSingle(int maxiterTime, bool gradientFK);

    void computeJacobian(
        Eigen::Vector3d& actuationPara, int maxiterTime, Eigen::VectorXd& JacobianIter);

    void computeInverseKinematics();
    Eigen::Vector3d& _shifttoOriginalPosAA();

    void inputTrajectoryPara();


private:

    Eigen::Matrix4d N;
    double terminalValue = 0.001; //0.0004;

    int minIter = 10;
    int maxIter; // max iteration time in single FK computing process

    int maxVariationIterNum = 10; // max terminal condition iteration

    double eps = 1e-7;
    double weightHard = 5.0;  // init 5.0
    double weightChamber = 3.0; // init 3.0

    int EachCore; // OpenMP parameter
    bool AAStopFlag = false;
    int afterStopIterNum = 0;


    QMeshPatch* model;
    int eleNum, nodeNum;

    Eigen::MatrixXd nodePos, nodePosInit, tetTable;
    Eigen::VectorXd tetSelected, weight;

    int select_num;
    Eigen::MatrixXi shiftTable;
    Eigen::MatrixXd shiftInit; // shift area node pos
    Eigen::Vector3d shiftPosInit; // shift area center pos
    Eigen::MatrixXd centerTop;

    Eigen::Vector3d tipPosInit;

    Eigen::SparseMatrix<double> matAT;
    Eigen::SparseMatrix<double> matATA;

    Eigen::SimplicialLDLT <Eigen::SparseMatrix<double>> Solver;

    std::vector<Eigen::MatrixXd> LocalCoord, InverseP, LocalGoal;
    std::vector<Eigen::VectorXd> VectorXPosition, VectorBSide, VectorXPosition_Last;

    double* SVDInputMatrix;	double* SVDOutputMatrix;

    void _compLocalGoalandInverse();
    void _buildMatA();
    void buildSystemAA(int iterTime, bool IKCompute);

    Eigen::Vector3d& _shifttoInitialPos();

    void _updateMeshPos();

private:

    std::vector<double> energyAAIter;

    std::vector<Eigen::MatrixXd> AA_Qset;
    std::vector<Eigen::MatrixXd> AA_Fset;
    std::vector<Eigen::MatrixXd> AA_Gset;
    std::vector<Eigen::MatrixXd> P_AA;
    Eigen::MatrixXd Q_LG;
    Eigen::MatrixXd iterQ;

    Eigen::MatrixXd FKPos;

    double Eperv;
    int m = 6; // parameter for AA

    void _localStepAA(
        Eigen::MatrixXd& inputQ, std::vector<Eigen::MatrixXd>& P_AA);
    void _globalStepAA(
        std::vector<Eigen::MatrixXd>& P_AA, Eigen::MatrixXd& outputQ);
    //void _updateQ(Eigen::MatrixXd& iterQ);

    double _calEnergyIterAA(Eigen::MatrixXd& inputQ, std::vector<Eigen::MatrixXd>& P_AA);
    double calEnergyIter();
    //void _localStep();
    //void _globalStep();

    /* program used in grasping project */
public:
    double fingerPressure;

    void buildSimulationSystem_grasping(double pressure);
    void buildSimulationSystem_doubleFinger(double pressure);


    void _compLocalGoalandInverse_grasping();
    bool initSolver_grasping(int iterTime);
    void updateExpandRatio_grasping(double actuationPara);

};
