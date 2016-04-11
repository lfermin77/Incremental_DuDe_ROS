//------------------------------------------------------------------------------
//  Copyright 2010-2011 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _MKSUM_DRAW_H_
#define _MKSUM_DRAW_H_

#ifdef WIN32
extern "C"{
#include "triangulate.h"
}
#else
#include "triangulate.h"
#endif

#include "compile_option.h"


//-----------------------------------------------------------------------------
#if GL_RENDERING
#include "gli.h"
#include "gliFont.h"
#endif

#include "bpc.h"
#include "cdt2.h"
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#include "SE2d.h"
#include "SE2d_data.h"
#include "SE2d_decomp.h"
//-----------------------------------------------------------------------------

#include "holediag.h"

#include "dude_param.h"

#include "draw_option.h"
#include "draw_decoration.h"

extern Draw_Options draw_options;
extern Draw_Decoration draw_decoration;

extern Dude_Param dude_param;

extern c_dude dude;

extern double average;

//-----------------------------------------------------------------------------
//
// Draw Minkowski sum
//
//-----------------------------------------------------------------------------
#if GL_RENDERING

void drawPoly(const c_polygon& P, bool negative);
void drawFill(c_polygon& pl, bool negative);
void drawPly(c_ply& P);
void drawFilledPly(c_ply& P);

void drawSkeletons(const list<se_m*>& l);

//void drawPlyLine(const c_plyline& PLine);

inline void drawCircle(const Point2d& p, float r)
{
    const int n_sides=16;
    const float res=PI2/n_sides;

    glPushMatrix();
    glTranslatef(0,0,0.1);
    glBegin( GL_POLYGON );
    for( float i = 0; i < PI2 ; i += res )
        glVertex2f( p[0]+ sin( i ) * r, p[1]+ cos( i ) * r );
    glEnd();

    glTranslatef(0,0,0.1);
    glPushAttrib(GL_CURRENT_BIT);
    glBegin( GL_LINE_LOOP );
    glColor3i(0,0,0);
    for( float i = 0; i < PI2; i += res )
        glVertex2f( p[0]+ sin( i ) * r, p[1]+ cos( i ) * r );
    glEnd();
    glPopAttrib();

    glPopMatrix();
}

void drawRefCircle()
{
    glLineWidth(2);
    glTranslatef(0,0,100);
    float radius=(dude_param.concavity_tau*gli::getScale())/2;
    glColor3d(0,1,0);
    drawCircle(Point2d( draw_decoration.box[1]-radius, draw_decoration.box[3]-radius), radius);

}
//draw the segmented polygon pieces
void drawDecomposedPolygonpieces()
{
	vector<c_polygon*>::iterator pit = draw_decoration.resultPolygons.begin();
	for (;pit!=draw_decoration.resultPolygons.end();++pit)
	{
		float r1 = rand()*1.0/RAND_MAX;
		float b1 = rand()*1.0/RAND_MAX;
		float g1 = rand()*1.0/RAND_MAX;
		glColor3f(r1,b1,g1);
		c_polygon& tmpPlygon = *(*pit);
		drawFill(tmpPlygon,false);
		//glLineWidth(10.0);
		//drawPoly(tmpPlygon,false);
	}
}
void drawFilledPly(c_ply& P)
{
	if(P.getSize()==0)
		return;
	glBegin(GL_POLYGON);
	ply_vertex* ptr = P.getHead();
	do 
	{
		const Point2d& pos=ptr->getPos();
		glVertex3f(pos[0], pos[1], 0);
		ptr=ptr->getNext();
	}while(ptr!=P.getHead());
	glEnd();	
}
void drawPoly(const c_ply& P, bool negative=false)
{
	ply_vertex * ptr=P.getHead();
	do{
		const Point2d& pos=ptr->getPos();
		if(!negative)
		    glVertex3f(pos[0], pos[1], 0);
		else
		    glVertex3f(-pos[0], -pos[1], 0);
		ptr=ptr->getNext();
	}while(ptr!=P.getHead());
}

inline void drawPoly(const c_polygon& poly, bool negative=false)
{
    for(c_polygon::const_iterator i=poly.begin();i!=poly.end();i++)
    {
        if(i==poly.begin()){
            glDisable(GL_LINE_STIPPLE);
        }
        else{
            glLineStipple(1, 0xAAAA);
            glEnable(GL_LINE_STIPPLE);
        }

        glBegin(GL_LINE_LOOP);
        drawPoly(*i, negative);
        glEnd();
    }

    glDisable(GL_LINE_STIPPLE);
}

//draw edge normals of P
inline void drawNormals(const c_ply& P)
{
    ply_vertex * ptr=P.getHead();
    do{
        const Point2d& p1=ptr->getPos();
        const Point2d& p2=ptr->getNext()->getPos();
        const Vector2d& n=ptr->getNormal();
        Point2d mid1((p1[0]+p2[0])/2,(p1[1]+p2[1])/2);
        Point2d mid2(mid1[0]+n[0]*draw_decoration.normal_length,mid1[1]+n[1]*draw_decoration.normal_length);

        glVertex3f(mid1[0], mid1[1], 0);
        glVertex3f(mid2[0], mid2[1], 0);
        ptr=ptr->getNext();
    }while(ptr!=P.getHead());
}

inline void drawNormals(const c_polygon& poly)
{
    glBegin(GL_LINES);
    for(c_polygon::const_iterator i=poly.begin();i!=poly.end();i++)
        drawNormals(*i);
    glEnd();
}


///////////////////////////////////////////////////////////////////////////

void drawFill(c_polygon& pl, bool negative=false)
{
    typedef c_polygon::iterator   PIT;

    int ringN=pl.size();             //number of rings
    int * ringVN=new int[ringN];     //number of vertices for each ring
    assert(ringVN);

    int vN=0;             //total number of vertices
    {
        int i=0;
        for(PIT ip=pl.begin();ip!=pl.end();ip++,i++){
            vN+=ip->getSize();
            ringVN[i]=ip->getSize();
        }
    }

    if( vN<3 ) return;
    int tN=(vN-2)+2*(ringN-1);       //(n-2)+2*(#holes)
    double * V=new double[vN*2];     //to hole vertices pos
    int *T=new int[3*tN];            //to hole resulting triangles
    assert(T);

    //copy vertices
    {   int i=0;
        for(PIT ip=pl.begin();ip!=pl.end();ip++){
            ply_vertex * ptr=ip->getHead();
            do{
                Point2d pt=ptr->getPos();
                V[i*2]=pt[0];
                V[i*2+1]=pt[1];
                ptr=ptr->getNext();
                i++;
            }while( ptr!=ip->getHead() );
        }
    }

    FIST_PolygonalArray(ringN, ringVN, (double (*)[2])V, &tN, (int (*)[3])T);
    {
        glBegin(GL_TRIANGLES);
        for(int i=0;i<tN;i++){
            for(int j=0;j<3;j++){
                int tid=T[i*3+j];
                if(!negative)
                    glVertex2d(V[tid*2],V[tid*2+1]);
                else
                    glVertex2d(-V[tid*2],-V[tid*2+1]);
            }
        }
        glEnd();
    }

    delete [] ringVN;
    delete [] V;
    delete [] T;
}


//-----------------------------------------------------------------------------
//
// Draw Alpha Decomp Stuff
//
//-----------------------------------------------------------------------------
void drawDiagonals_few(const vector<c_diagonal>& diagonals)
{
	int id = 0;

    typedef vector<c_diagonal> DIAS;
    glBegin(GL_LINES);
    for(DIAS::const_iterator i=diagonals.begin();i!=diagonals.end();i++){

    	id++;
    	if(id != 48 && id != 67 && id != 47)
    		continue;

//    	if(i->inHole)
//    	{
//    		glColor3d(1.0,1.0,1.0);
//    		glLineWidth(5);
//    	}
//    	else{
//    		glColor3d(0.85f,0.85f,0.5f);
//    	}
        const Point2d& p1=i->getV1()->getPos();
        const Point2d& p2=i->getV2()->getPos();

        glVertex2dv(p1.get());
        glVertex2dv(p2.get());
    }
    glEnd();
}

void drawDiagonals(const vector<c_diagonal>& diagonals)
{

    typedef vector<c_diagonal> DIAS;
    glBegin(GL_LINES);
    for(DIAS::const_iterator i=diagonals.begin();i!=diagonals.end();i++){

//    	if(i->inHole)
//    	{
//    		glColor3d(1.0,1.0,1.0);
//    		glLineWidth(5);
//    	}
//    	else{
//    		glColor3d(0.85f,0.85f,0.5f);
//    	}
        const Point2d& p1=i->getV1()->getPos();
        const Point2d& p2=i->getV2()->getPos();

        glVertex2dv(p1.get());
        glVertex2dv(p2.get());
    }
    glEnd();
}

void drawBridge(c_BPC * bpc, float average)
{
    ply_vertex * s=bpc->getSource1();
    ply_vertex * t=bpc->getSource2();

    //draw bridge
    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3d(0,0.5,.5);
    glVertex2dv(s->getPos().get());
    glVertex2dv(t->getPos().get());
    glEnd();
    glLineWidth(1);

    //draw end points as dots
    glTranslated(0,0,0.1);
    glColor3d(0,0.5,.5);
    drawCircle(s->getPos(),average/2);
    drawCircle(t->getPos(),average/2);

    //draw kids bridge
    const list<c_BPC *>& kids=bpc->getKids();
    for(list<c_BPC *>::const_iterator i=kids.begin();i!=kids.end();i++){
        c_BPC * kid=*i;
        drawBridge(kid,average/2);
    }
}

void drawVID(vector<ply_vertex *>& vl)
{
    //
    char tmp[36];
    glColor3d(0,0,0);
    for(vector<ply_vertex *>::const_iterator i=vl.begin();i!=vl.end();i++)
    {
        const Point2d& pos=(*i)->getPos();
        sprintf(tmp,"%d",(int)((*i)->getVID()));
        drawstr(pos[0],pos[1],5,tmp);
    }
}

void drawBPC(c_BPC * bpc, float largest_concavity, float smallest_concavity)
{

    float scale_avg=average;
    float pR=getP().front().getRadius();

    glPushMatrix();

    //draw bridges between sources
    if(draw_options.showBridges)
    {
        drawBridge(bpc,average/2);
    }


/*
    //draw concavity
    if(showConcavity)
    {
        glTranslated(0,0,0.1);
        glColor3d(1,0,0);


        const vector<ply_vertex *>& pms= VIP_vertices; //bpc->getConcavities();
        for(vector<ply_vertex *>::const_iterator i=pms.begin();i!=pms.end();i++)
        {
            float c=(*i)->getExtra().concavity/pR;
            const Point2d& pos=(*i)->getPos();
            scale_avg=average*(c-smallest_concavity+0.2)/(largest_concavity-smallest_concavity+0.2)*2;
            drawCircle(pos,scale_avg);
        }

    }
*/

    glPopMatrix();

}

void drawBPCs()
{
    c_dude& ap=dude;
    const list<c_BPC *>& bpcs=ap.getBPS();
    float pR=getP().front().getRadius();

    //do this one time process..
    typedef list<c_BPC *>::const_iterator IT;
    static float smallest_concavity=draw_decoration.MINCONCAVITY/pR;//         FLT_MAX;
    static float largest_concavity=draw_decoration.MAXCONCAVITY/pR;

//    if(smallest_concavity==FLT_MAX)
//    {
//        for(IT i=bpcs.begin();i!=bpcs.end();i++){
//            const list<ply_vertex *>& pms=(*i)->getConcavities();
//            for(list<ply_vertex *>::const_iterator j=pms.begin();j!=pms.end();j++){
//                float c=(*j)->getExtra().concavity;
//                if(c<smallest_concavity){
//                    smallest_concavity=c;
//                }
//                if(c>largest_concavity){
//                    largest_concavity=c;
//                }
//            }
//        }
//
//        const vector<ply_vertex *>& holefp=allFeaturePMs;//dude.holeFeatPnts;
//        for(vector<ply_vertex *>::const_iterator j=holefp.begin();j!=holefp.end();j++){
//            if( (*j)->getExtra().isPM()==false || (*j)->getExtra().diagonals.empty() ) continue;
//            float c=(*j)->getExtra().concavity;
//            if(c<smallest_concavity){
//                smallest_concavity=c;
//            }
//            if(c>largest_concavity){
//                largest_concavity=c;
//            }
//        }
//
//        largest_concavity/=pR;
//        smallest_concavity/=pR;
//    }

    //
    //now we are ready to draw
    //

    if(draw_options.showConstraints){
        glTranslatef(0,0,1);
        glColor3d(0.3,0.3,0.3);
        drawPoly(ap.getSimplifiedPolygon());

        glLineWidth(1);

        //for testing evaluation
//        for(vector<RiBox*>::iterator bit = gridBoxes.begin(); bit != gridBoxes.end(); ++bit)
//        {
//        	RiBox* tb = *bit;
//        	glBegin(GL_POLYGON);
//        	glVertex2f(tb->minx, tb->miny);
//        	glVertex2f(tb->minx, tb->maxy);
//        	glVertex2f(tb->maxx, tb->maxy);
//        	glVertex2f(tb->maxx, tb->miny);
//        	glEnd();
//        }

		glLineWidth(1);
        glColor3d(0.0,0.0,1.0);
        glBegin(GL_LINES);
        //for testing evaluation
         for(vector<c_diagonal>::iterator dit = dude.holeCutDiagonals.begin(); dit != dude.holeCutDiagonals.end(); ++dit)
         {
        	 c_diagonal& diag = *dit;
         	glVertex2f(diag.getV1()->getPos()[0], diag.getV1()->getPos()[1]);
         	glVertex2f(diag.getV2()->getPos()[0], diag.getV2()->getPos()[1]);

         }
         glEnd();


         if(draw_decoration.holecutRoot.getV1()!=NULL)
         {
        	 cout<<"drawing hole cut root...."<<endl;
        	 glLineWidth(4);
        	 glColor3d(1.0,0.6,0.3);
        	 glBegin(GL_LINES);
        	 	 glVertex2f(draw_decoration.holecutRoot.getV1()->getPos()[0], draw_decoration.holecutRoot.getV1()->getPos()[1]);
        	 	 glVertex2f(draw_decoration.holecutRoot.getV2()->getPos()[0], draw_decoration.holecutRoot.getV2()->getPos()[1]);

        	 glEnd();
         }
         		glLineWidth(1);
                 glColor3d(0.0,0.0,1.0);
                 glBegin(GL_LINES);
                 //for testing evaluation
                  for(vector<pair<Point2d,Point2d> >::iterator dit = draw_decoration.holeCutTreeLines.begin(); dit != draw_decoration.holeCutTreeLines.end(); ++dit)
                  {
                	  pair<Point2d,Point2d>& diag = *dit;
                  	glVertex2f(diag.first[0], diag.first[1]);
                  	glVertex2f(diag.second[0],diag.second[1]);

                  }
                  glEnd();



    }

    if(draw_options.showRepUserCuts)
    {
    	glLineWidth(4);
		glColor3d(1.0,0.0,1.0);
		glBegin(GL_LINES);
		for(vector<c_diagonal>::iterator dit = draw_decoration.repDiags.begin(); dit != draw_decoration.repDiags.end(); ++dit)
		{
			c_diagonal& diag = *dit;
			glVertex2f(diag.getV1()->getPos()[0], diag.getV1()->getPos()[1]);
			glVertex2f(diag.getV2()->getPos()[0], diag.getV2()->getPos()[1]);
		}
		glEnd();
    }

    //if(draw_options.showCompCuts)
    if(false)
    {
    	glLineWidth(4);
		glColor3d(1.0, 1.0, 0.0);
		glBegin(GL_LINES);
		for(vector<c_diagonal>::iterator dit = draw_decoration.compDiags.begin(); dit != draw_decoration.compDiags.end(); ++dit)
		{
			c_diagonal& diag = *dit;
			glVertex2f(diag.getV1()->getPos()[0], diag.getV1()->getPos()[1]);
			glVertex2f(diag.getV2()->getPos()[0], diag.getV2()->getPos()[1]);
		}
		glEnd();
    }

    if(draw_options.showAllUserCutsInClustering)
    {
    	 static float colors[][3] = {
    	        		{0.5, 1.0, 0.5}, {1.0, 1.0, 0.5}, {1.0, 0.5, 0.5}, {0.5, 1.0, 1.0}, {1.0, 0.5, 1.0},
    	        		{0.5, 0.75, 0.5}, {0.75, 0.75, 0.5}, {0.75, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.75, 0.5, 0.75},
    	        		{0.25, 1.0, 0.25}, {1.0, 1.0, 0.25}, {1.0, 0.25, 0.25}, {0.25, 1.0, 1.0}, {1.0, 0.25, 1.0},
    	        		{0.25, 0.75, 0.25}, {0.75, 0.75, 0.25}, {0.75, 0.25, 0.25}, {0.25, 0.75, 0.75}, {0.75, 0.25, 0.75},
    	        };

		glLineWidth(2);
		uint size = draw_decoration.userDiags.size();
		for(uint i=0;i<size;i++)
		{
			 glColor3fv(colors[i]);
			 glBegin(GL_LINES);
			 for(vector<c_diagonal>::iterator dit = draw_decoration.userDiags[i].begin(); dit != draw_decoration.userDiags[i].end(); ++dit)
			 {
				c_diagonal& diag = *dit;
				glVertex2dv(diag.getV1()->getPos().get());
				glVertex2dv(diag.getV2()->getPos().get());

			 }
			 glEnd();
		}
    }

    if(draw_options.showAllUserCuts)
    {
    	glLineWidth(3);
		uint size = draw_decoration.userDiags.size();

		static float alphas[] = {0.3, 0.05, 0.1, 1.0};

		float alpha_factor = 1.0 / sqrt(size);

		for(uint i=0;i<size;i++)
		{
			 glBegin(GL_LINES);
			 for(vector<c_diagonal>::iterator dit = draw_decoration.userDiags[i].begin(); dit != draw_decoration.userDiags[i].end(); ++dit)
			 {
				glColor4f(0, 0, 1.0, alphas[dit->getQuality()]*alpha_factor);
				c_diagonal& diag = *dit;
				glVertex2dv(diag.getV1()->getPos().get());
				glVertex2dv(diag.getV2()->getPos().get());

			 }
			 glEnd();
		}
    }

    if(draw_options.showDecompose)
    {
        //draw finally cuts
        glLineWidth(2);
        glTranslatef(0,0,1);
        glColor3d(1,0,0);
        drawDiagonals(draw_decoration.allAccumulatedCuts);
        //drawDiagonals(ap.getFinalCuts());//changed by Guilin 04/10/2013
        glLineWidth(1);
    }

    if(draw_options.showFewCut)
    {
        //draw finally cuts
        glLineWidth(2);
        glTranslatef(0,0,1);
        glColor3d(1,0,0);
        drawDiagonals_few(draw_decoration.allAccumulatedCuts);
        //drawDiagonals(ap.getFinalCuts());//changed by Guilin 04/10/2013
        glLineWidth(3);
    }


    for(IT i=bpcs.begin();i!=bpcs.end();i++)
        drawBPC(*i,largest_concavity,smallest_concavity);

    if(draw_options.showVertexID) drawVID(draw_decoration.VIP_vertices);

    //draw the selected PM if it's not null
    if(draw_decoration.g_selected_PM!=NULL)
    {
        glTranslatef(0,0,1);
        glLineWidth(4);
        vector<c_cutset>& csets=draw_decoration.g_selected_PM->getExtra().cutsets;
        uint cset_size=csets.size();
        for(uint j=0;j<cset_size;j++){
            if(csets[j].getType()==c_cutset::SINGLE_PM){
                glColor3d(0.85f,0.5f,0.5f);
            }
            else if(csets[j].getType()==c_cutset::SINGLE_TIP ){
                glColor3d(0.85f,0.85f,0.5f);
            }
            else
                glColor3d(0.5f,0.5f,0.85f);
            drawDiagonals(csets[j].getDiagonals());
        }
        glLineWidth(1);

        glTranslatef(0,0,1);
        float c=draw_decoration.g_selected_PM->getExtra().concavity/pR;
        double scale_avg=average*(c-smallest_concavity+0.2)/(largest_concavity-smallest_concavity+0.2)*2;
        glColor3d(1,1,0);
        drawCircle(draw_decoration.g_selected_PM->getPos(),scale_avg);
    }

    if(draw_options.showConcavity)
    {
    	//cerr<<"max concavity and min concavity "<<largest_concavity<<"\t"<<smallest_concavity<<endl;
    	float scale_avg=average;
    	float pR=getP().front().getRadius();
        glTranslated(0,0,0.1);
        glColor3d(1,0,0);
        const vector<ply_vertex *>& pms=draw_decoration.allFeaturePMs;
        vector<float>::const_iterator cit = draw_decoration.allFeatureConcavities.begin();
        for(vector<ply_vertex *>::const_iterator i=pms.begin();i!=pms.end();i++,++cit)
        {
            //float c=(*i)->getExtra().concavity/pR;
        	float c = *cit;
            const Point2d& pos=(*i)->getPos();
            scale_avg=average*(c-smallest_concavity+0.2)/(largest_concavity-smallest_concavity+0.2)*2;
            //cerr<<scale_avg<<endl;
            drawCircle(pos,scale_avg);
        }

    }

}

#include "polyline.h"
extern c_plyline g_hullply;
//extern list<ply_vertex *> g_hullply;
extern list<Point2d> g_points;

void drawAll()
{
	float radius = getP().front().getRadius();
	average = 0.02*radius;

    glLineWidth(1);

    //draw P
    if(draw_options.showP)
    {
        glTranslatef(0,0,1);
        //glPushMatrix();
        glColor3f(0.7,0.75,0.75);
        drawFill(getP());

        glTranslatef(0,0,1);
        glColor3f(0,0,0);
        drawPoly(getP());

        if(draw_options.showNormal){
            glTranslatef(0,0,1);
            glColor3f(0,0,0);
            drawNormals(getP());
        }
    }

    drawBPCs();

	if(draw_options.showPolygonPieces)
		drawDecomposedPolygonpieces();

	if(draw_options.showTauCircle)
	    drawRefCircle();

	if(draw_options.showSkeleton)
	{
		drawSkeletons(draw_decoration.se.getTodoList());
	}

}


/**********************************************************************************************************/
void drawSkeleton(const SKG& G)
{
	vector<pair<pair<sk_node,sk_node>,sk_edge > > edges ;
	int esize = G.GetEdgesVData(edges);

	for(int ie = 0; ie != esize; ie++){
		Point2d p1 = edges[ie].first.first.m_pos;
		Point2d p2 = edges[ie].first.second.m_pos;
		glBegin(GL_LINES);
		glVertex2d(p1[0], p1[1]);
		glVertex2d(p2[0], p2[1]);
		glEnd();
	}
}
void drawSkeletons(const list<se_m*>& l)
{
	typedef list<se_m*>::const_iterator MIT;
	glLineWidth(2.5);
	glColor3d(0,0,0.5);
	for(MIT im = l.begin();im!=l.end();im++){
		drawSkeleton((*im)->skeleton);
	}
}

#endif

#endif //_MKSUM_DRAW_H_


