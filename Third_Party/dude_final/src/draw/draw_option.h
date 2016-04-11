
#ifndef _DRAW_OPTION_H_
#define _DRAW_OPTION_H_

///////////////////////////////////////////////////
//the drawing options


#include <string>
using namespace std;

struct Draw_Options
{
	Draw_Options()
	{
		showDecompose = true;
		showConcavity = false;
		showP = true;
		showNormal = false;
		showCuts = false;
		showConstraints = false;
		showCompCuts = false;
		showBridges = false;
		saveImg = false;
		savePDF = false;
		savsPS = false;


		showAllBridges = false;
		showTauCircle = true;
		showPolygonPieces = false;
		showVertexID = false;
		showSkeleton = false;
		vizSource = "internal";
		visulizeSegment = false;
		showAllUserCutsInClustering = false;
		showAllUserCuts = false;
		showRepUserCuts = false;
		useDude2d = true;
		outputSegFile = false;

		showPolygonPieceForPS = false;
		showSkeletonForPS = false;


		showGL = true;
		neg_P = false;

		//debug
		showFewCut = true;
	}
	//-----------------------------------------------------------------------------
	//variables used in rendering
	int showDecompose;
	int showConcavity;
	int showP;
	int showNormal;
	int showCuts;
	int showConstraints;
	int showCompCuts;
	int showBridges;
	int saveImg;
	int savePDF;
	int savsPS;


	int showAllBridges;
	int showTauCircle;
	int showPolygonPieces;
	int showVertexID;
	int showSkeleton;
	string vizSource;
	int visulizeSegment;
	int showAllUserCutsInClustering;
	int showAllUserCuts;
	int showRepUserCuts;
	int useDude2d;
	int outputSegFile;

	int showPolygonPieceForPS;
	int showSkeletonForPS;

	//general

	bool showGL;
	bool neg_P;

	//debug
	int showFewCut;
};





#endif
