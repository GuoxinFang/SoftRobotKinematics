// QMeshNode.cpp: implementation of the QMeshNode class.
//
//////////////////////////////////////////////////////////////////////

#include <math.h>
#include "QMeshPatch.h"
#include "QMeshTetra.h"
#include "QMeshFace.h"
#include "QMeshEdge.h"
#include "QMeshNode.h"
#include <iostream>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

QMeshNode::QMeshNode()
{
    indexno=0;		m_trackingFace=nullptr;
    m_meanCurvatureNormalVector[0]=0.0;
    m_meanCurvatureNormalVector[1]=0.0;
    m_meanCurvatureNormalVector[2]=0.0;
    m_gaussianCurvature=0.0;	m_pMaxCurvature=0.0;	m_pMinCurvature=0.0;
    m_boundaryDist=0.0;
	faceList.RemoveAll();
	edgeList.RemoveAll();
	nodeList.RemoveAll();
	for(int i=0;i<8;i++) flags[i]=false;

    m_trackingPos[0]=0.0;
    m_trackingPos[1]=0.0;
    m_trackingPos[2]=0.0;
    attachedPointer=nullptr;
}

QMeshNode::~QMeshNode()
{
	faceList.RemoveAll();
	edgeList.RemoveAll();
	nodeList.RemoveAll();
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

int QMeshNode::GetIndexNo() 
{
	return indexno;
}
	
void QMeshNode::SetIndexNo( const int _index )
{
	indexno=_index;
}

bool QMeshNode::GetAttribFlag( const int whichBit )
{
	return flags[whichBit];
}

void QMeshNode::SetAttribFlag( const int whichBit, const bool toBe )
{
	flags[whichBit]=toBe;
}

void QMeshNode::GetCoord2D( double &x, double &y )
{
	x=coord2D[0];	y=coord2D[1];
}

void QMeshNode::SetCoord2D( double x, double y )
{
	coord2D[0]=x;	coord2D[1]=y;
}

void QMeshNode::GetCoord3D( double &x, double &y, double &z )
{
	x=coord3D[0];	y=coord3D[1];	z=coord3D[2];
}

void QMeshNode::GetCoord3D(Eigen::Vector3d& pos)
{
    pos(0) = coord3D[0];	pos(1) = coord3D[1];	pos(2) = coord3D[2];
}

void QMeshNode::SetCoord3D( double x, double y, double z )
{
	coord3D[0]=x;	coord3D[1]=y;	coord3D[2]=z;
}

void QMeshNode::SetCoord3D(Eigen::Vector3d& pos)
{
    coord3D[0] = pos(0);	coord3D[1] = pos(1); coord3D[2] = pos(2);
}

void QMeshNode::GetCoord3D_last( double &x, double &y, double &z )
{
	x=coord3D_last[0];	y=coord3D_last[1];	z=coord3D_last[2];
}

void QMeshNode::SetCoord3D_last( double x, double y, double z )
{
	coord3D_last[0]=x;	coord3D_last[1]=y;	coord3D_last[2]=z;
}

void QMeshNode::SetMeanCurvatureNormalVector(double kHx, double kHy, double kHz)
{
    m_meanCurvatureNormalVector[0]=kHx;
    m_meanCurvatureNormalVector[1]=kHy;
    m_meanCurvatureNormalVector[2]=kHz;
}

void QMeshNode::GetMeanCurvatureNormalVector(double &kHx, double &kHy, double &kHz)
{
    kHx=m_meanCurvatureNormalVector[0];
    kHy=m_meanCurvatureNormalVector[1];
    kHz=m_meanCurvatureNormalVector[2];
}

void QMeshNode::SetGaussianCurvature(double kG)
{
    m_gaussianCurvature=kG;
}

double QMeshNode::GetGaussianCurvature()
{
    return m_gaussianCurvature;
}

void QMeshNode::SetPMaxCurvature(double k1)
{
    m_pMaxCurvature=k1;
}

double QMeshNode::GetPMaxCurvature()
{
    return m_pMaxCurvature;
}

void QMeshNode::SetPMinCurvature(double k2)
{
    m_pMinCurvature=k2;
}

double QMeshNode::GetPMinCurvature()
{
    return m_pMinCurvature;
}

void QMeshNode::SetMinCurvatureVector(double vx, double vy, double vz)
{
    m_minCurvatureVector[0]=vx;	m_minCurvatureVector[1]=vy;	m_minCurvatureVector[2]=vz;
}

void QMeshNode::GetMinCurvatureVector(double &vx, double &vy, double &vz)
{
    vx=m_minCurvatureVector[0];	vy=m_minCurvatureVector[1];	vz=m_minCurvatureVector[2];
}

void QMeshNode::SetMaxCurvatureVector(double vx, double vy, double vz)
{
    m_maxCurvatureVector[0]=vx;	m_maxCurvatureVector[1]=vy;	m_maxCurvatureVector[2]=vz;
}

void QMeshNode::GetMaxCurvatureVector(double &vx, double &vy, double &vz)
{
    vx=m_maxCurvatureVector[0];	vy=m_maxCurvatureVector[1];	vz=m_maxCurvatureVector[2];
}

void QMeshNode::SetBoundaryDis(double dist)
{
    m_boundaryDist=dist;
}

double QMeshNode::GetBoundaryDis()
{
    return m_boundaryDist;
}

void QMeshNode::CalNormal()
{
    double nx, ny, nz, tt;
    nx=0.0;	ny=0.0;	nz=0.0;

    GLKPOSITION Pos;
    for(Pos=faceList.GetHeadPosition();Pos!=nullptr;)
    {
        double a,b,c,d;
        QMeshFace *temp=(QMeshFace *)(faceList.GetNext(Pos));
		if (temp->inner == true) continue;
        temp->GetPlaneEquation(a,b,c,d);
        nx+=a;	ny+=b;	nz+=c;
    }
    tt=nx*nx+ny*ny+nz*nz;
    tt=sqrt(tt);

    m_normal[0]=(double)(nx/tt);	m_normal[1]=(double)(ny/tt);	m_normal[2]=(double)(nz/tt);
}

void QMeshNode::CalNormal(double normal[])
{
    double nx, ny, nz, tt;
	nx=0.0;	ny=0.0;	nz=0.0;

	GLKPOSITION Pos;
    for(Pos=faceList.GetHeadPosition();Pos!=nullptr;)
	{
		double a,b,c,d;
		QMeshFace *temp=(QMeshFace *)(faceList.GetNext(Pos));
		if (temp->inner == true) continue;
		temp->GetPlaneEquation(a,b,c,d);
		//std::cout << a << b << c << d << std::endl;
		nx+=a;	ny+=b;	nz+=c;
	}
	tt=nx*nx+ny*ny+nz*nz;
	tt=sqrt(tt);

    m_normal[0]=(double)(nx/tt);	m_normal[1]=(double)(ny/tt);	m_normal[2]=(double)(nz/tt);
    normal[0]=m_normal[0]; normal[1]=m_normal[1]; normal[2]=m_normal[2];
}

void QMeshNode::SetMeshPatchPtr(QMeshPatch* _mesh)
{
	meshSurface=_mesh;
}

QMeshPatch* QMeshNode::GetMeshPatchPtr()
{
	return meshSurface;
}

void QMeshNode::AddTetra(QMeshTetra *trglTetra)
{
	tetraList.AddTail(trglTetra);
}

int QMeshNode::GetTetraNumber()
{
	return tetraList.GetCount();
}

QMeshTetra* QMeshNode::GetTetraRecordPtr(int No) //from 1 to n
{
	if ((No < 1) || (No > tetraList.GetCount()))    return  NULL;
	return (QMeshTetra *)tetraList.GetAt(tetraList.FindIndex(No - 1));
}

GLKObList& QMeshNode::GetTetraList()
{
	return tetraList;
}

void QMeshNode::AddFace(QMeshFace *_face)
{
	faceList.AddTail(_face);
}

int QMeshNode::GetFaceNumber() 
{
	return faceList.GetCount();
}

QMeshFace* QMeshNode::GetFaceRecordPtr(int No) 	//from 1 to n
{
    if( (No < 1) || (No > faceList.GetCount()))    return  nullptr;
    return (QMeshFace *)faceList.GetAt(faceList.FindIndex(No-1));
}

GLKObList& QMeshNode::GetFaceList()
{
	return faceList;
}

void QMeshNode::AddEdge(QMeshEdge *_edge)
{
	edgeList.AddTail(_edge);
}

int QMeshNode::GetEdgeNumber() 
{
	return edgeList.GetCount();
}

QMeshEdge* QMeshNode::GetEdgeRecordPtr(int No) 	//from 1 to n
{
    if( (No < 1) || (No > edgeList.GetCount()))    return  nullptr;
    return (QMeshEdge *)edgeList.GetAt(edgeList.FindIndex(No-1));
}

GLKObList& QMeshNode::GetEdgeList()
{
	return edgeList;
}

void QMeshNode::AddNode(QMeshNode *_node)
{
	nodeList.AddTail(_node);
}

int QMeshNode::GetNodeNumber()
{
	return nodeList.GetCount();
}

bool QMeshNode::IsNodeInNodeList(QMeshNode *_node)
{
	GLKPOSITION Pos;

    for(Pos=nodeList.GetHeadPosition();Pos!=nullptr;) {
		QMeshNode *tempnode=(QMeshNode *)(nodeList.GetNext(Pos));
		if (tempnode==_node) return true;
	}

	return false;
}

QMeshNode* QMeshNode::GetNodeRecordPtr(int No)	//from 1 to n
{
    if ((No < 1) || (No > nodeList.GetCount())) return  nullptr;
    return (QMeshNode *)nodeList.GetAt(nodeList.FindIndex(No-1));
}

GLKObList& QMeshNode::GetNodeList()
{
	return nodeList;
}

void QMeshNode::GetCoord3D_FLP( double &x, double &y, double &z )
{
    x=coord3D_FLP[0];	y=coord3D_FLP[1];	z=coord3D_FLP[2];
}

void QMeshNode::SetCoord3D_FLP( double x, double y, double z )
{
   coord3D_FLP[0]=x;	coord3D_FLP[1]=y;	coord3D_FLP[2]=z;
}

//bool QMeshPatch::inputOBJFile(char* filename, bool bOBTFile)
//{
//	FILE *fp;
//	char fields[MAX_EDGE_NUM][255];
//	char linebuf[256], buf[100];
//	GLKPOSITION Pos;
//	GLKPOSITION PosNode;
//	int i;
//	QMeshNode *node, *startNode, *endNode;
//	QMeshEdge *edge;
//	QMeshFace *face;
//	QMeshNode **nodeArray;
//	float xx, yy, zz, ww;
//	//	float minX,maxX,minY,maxY,minZ,maxZ;
//
//	fp = fopen(filename, "r");
//	if (!fp) {
//		printf("===============================================\n");
//		printf("Can not open the data file - OBJ File Import!\n");
//		printf("===============================================\n");
//		return false;
//	}
//
//	ClearAll();
//	while (!feof(fp)) {
//		sprintf(buf, "");
//		sprintf(linebuf, "");
//		fgets(linebuf, 255, fp);
//		sscanf(linebuf, "%s", buf);
//
//		if ((strlen(buf) == 1) && (buf[0] == 'v'))
//		{
//			float rr, gg, bb;
//			rr = 1.0; gg = 1.0; bb = 1.0;
//			if (bOBTFile)
//				sscanf(linebuf, "%s %f %f %f %f\n", buf, &xx, &yy, &zz, &ww);
//			else
//				sscanf(linebuf, "%s %f %f %f %f %f %f\n", buf, &xx, &yy, &zz, &rr, &gg, &bb);
//			//			float scale=.75f;	xx=xx*scale;	yy=yy*scale;	zz=zz*scale;
//
//			node = new QMeshNode;
//			node->SetMeshPatchPtr(this);
//			node->SetCoord3D(xx, yy, zz);
//			node->SetCoord3D_last(xx, yy, zz);
//			node->SetIndexNo(nodeList.GetCount() + 1);
//			node->identifiedIndex = node->GetIndexNo();
//			node->m_nIdentifiedPatchIndex = -1;
//			node->selected = false;
//			if (bOBTFile)
//				node->SetWeight(ww);
//			else {
//				node->SetWeight(-1.0);
//				node->SetColor(rr, gg, bb);
//			}
//			nodeList.AddTail(node);
//		}
//	}
//	fclose(fp);
//
//	int nodeNum = nodeList.GetCount();
//	nodeArray = new QMeshNode*[nodeNum];
//	i = 0;
//	for (Pos = nodeList.GetHeadPosition(); Pos != NULL; i++) {
//		node = (QMeshNode*)(nodeList.GetNext(Pos));
//		nodeArray[i] = node;
//	}
//
//	fp = fopen(filename, "r");
//	while (!feof(fp)) {
//		sprintf(buf, "");
//		sprintf(linebuf, "");
//		fgets(linebuf, 255, fp);
//		sscanf(linebuf, "%s", buf);
//
//		if ((strlen(buf) == 1) && (buf[0] == 'f'))
//		{
//			char seps[] = " \r\n";
//			char seps2[] = "/";
//			char *token;
//			char linebuf2[255];
//			strcpy(linebuf2, linebuf);
//
//			int num = 0;
//			token = strtok(linebuf, seps);
//			while (nullptr != token) {
//				token = strtok(nullptr, seps);
//				num++;
//			}
//			num = num - 1;
//
//			if (num>MAX_EDGE_NUM) continue;
//			if (num<1) continue;
//
//			face = new QMeshFace;
//			face->SetMeshPatchPtr(this);
//			face->SetIndexNo(faceList.GetCount() + 1);
//			faceList.AddTail(face);
//
//			token = strtok(linebuf2, seps);
//			for (i = 0; i<num; i++) {
//				token = strtok(NULL, seps);
//				strcpy(fields[i], token);
//			}
//
//			bool bValid = true;
//			for (i = 0; i<num; i++) {
//				token = strtok(fields[i], seps2);
//				int nodeIndex = atoi(token);
//
//				//				double xc,yc,zc;
//				//				nodeArray[nodeIndex-1]->GetCoord3D(xc,yc,zc);
//				//				if (xc<0.0) bValid=false;
//
//				(face->GetAttachedList()).AddTail(nodeArray[nodeIndex - 1]);
//				//				(face->GetAttachedList()).AddHead(nodeArray[nodeIndex-1]);
//			}
//			if (!bValid) { delete face; faceList.RemoveTail(); continue; }
//
//			bool bDegenerated = false;
//			for (Pos = face->GetAttachedList().GetHeadPosition(); Pos != NULL;) {
//				QMeshNode *pNode = (QMeshNode *)(face->GetAttachedList().GetNext(Pos));
//				GLKPOSITION Pos2 = Pos;
//				for (; Pos2 != NULL;) {
//					QMeshNode *qNode = (QMeshNode *)(face->GetAttachedList().GetNext(Pos2));
//					if ((pNode == qNode)) {
//						bDegenerated = true;
//						break;
//					}
//				}
//				if (bDegenerated) break;
//			}
//			if (bDegenerated) {
//				faceList.RemoveTail();
//				delete face;
//			}
//		}
//	}
//	fclose(fp);
//
//	delete[]nodeArray;
//
//	//---------------------------------------------------------------------
//	//	Build the topology
//	//---------------------------------------------------------------------
//	//	Step 1: build the edges
//	for (Pos = faceList.GetHeadPosition(); Pos != NULL;) {
//		face = (QMeshFace*)(faceList.GetNext(Pos));
//
//		int edgeNum = (face->GetAttachedList()).GetCount();
//		face->SetEdgeNum(edgeNum);
//
//		//nodeArray=(QMeshNode**)new long[edgeNum];
//		nodeArray = new QMeshNode*[edgeNum];
//
//		i = 0;
//		for (PosNode = (face->GetAttachedList()).GetHeadPosition(); PosNode != NULL; i++) {
//			nodeArray[i] = (QMeshNode*)((face->GetAttachedList()).GetNext(PosNode));
//			(nodeArray[i]->GetFaceList()).AddTail(face);
//		}
//
//		for (i = 0; i<edgeNum; i++) {
//			edge = NULL;	startNode = nodeArray[i];	endNode = nodeArray[(i + 1) % edgeNum];
//			bool bDir;
//			for (PosNode = (startNode->GetEdgeList()).GetHeadPosition(); PosNode != NULL;) {
//				QMeshEdge *temp = (QMeshEdge *)((startNode->GetEdgeList()).GetNext(PosNode));
//				if ((temp->GetStartPoint() == startNode) && (temp->GetEndPoint() == endNode) && (temp->GetLeftFace() == NULL)) {
//					edge = temp;	bDir = true;
//				}
//				else if ((temp->GetStartPoint() == endNode) && (temp->GetEndPoint() == startNode) && (temp->GetRightFace() == NULL)) {
//					edge = temp;	bDir = false;
//				}
//			}
//			if (edge && bDir) {
//				face->SetEdgeRecordPtr(i, edge);
//				face->SetDirectionFlag(i, true);
//				edge->SetLeftFace(face);
//			}
//			else if (edge && (!bDir)) {
//				face->SetEdgeRecordPtr(i, edge);
//				face->SetDirectionFlag(i, false);
//				edge->SetRightFace(face);
//			}
//			else {
//				edge = new QMeshEdge;
//				edge->SetMeshPatchPtr(this);
//				edge->SetStartPoint(startNode);
//				edge->SetEndPoint(endNode);
//				edge->SetIndexNo(edgeList.GetCount() + 1);
//				edgeList.AddTail(edge);
//
//				edge->SetLeftFace(face);
//				face->SetEdgeRecordPtr(i, edge);
//				face->SetDirectionFlag(i, true);
//				(startNode->GetEdgeList()).AddTail(edge);
//				(endNode->GetEdgeList()).AddTail(edge);
//			}
//		}
//
//		delete[]nodeArray;
//		face->GetAttachedList().RemoveAll();
//	}
//	//---------------------------------------------------------------------
//	//	Step 2: compute the normal
//	for (Pos = faceList.GetHeadPosition(); Pos != NULL;) {
//		face = (QMeshFace*)(faceList.GetNext(Pos));
//		face->CalPlaneEquation();
//		double xx, yy, zz;
//		face->CalCenterPos(xx, yy, zz);
//		face->selected = false;
//		face->m_nIdentifiedPatchIndex = -1;
//	}
//	for (Pos = edgeList.GetHeadPosition(); Pos != NULL;) {
//		edge = (QMeshEdge*)(edgeList.GetNext(Pos));
//		edge->cableIndex = -1;
//		edge->seamIndex = -1;
//		edge->selected = false;
//		edge->CalLength();
//		edge->Cal2DLength();
//		if ((edge->GetLeftFace()) && (edge->GetRightFace())) continue;
//		edge->SetAttribFlag(0);
//		edge->GetStartPoint()->SetAttribFlag(0);
//		edge->GetEndPoint()->SetAttribFlag(0);
//	}
//	std::cout << "Finish input obj" << std::endl;
//	return true;
//}
