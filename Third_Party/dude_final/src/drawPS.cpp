#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#if PS_RENDERING

#include "compile_option.h"
//-----------------------------------------------------------------------------
#include "polygon.h"
#include "bpc.h"
#include "cdt2.h"
//-----------------------------------------------------------------------------


#include <libps/pslib.h>
//-----------------------------------------------------------------------------
#include "SE2d.h"
#include "SE2d_data.h"
#include "SE2d_decomp.h"
//-----------------------------------------------------------------------------

#include "draw_option.h"
#include "draw_decoration.h"

extern Draw_Options draw_options;
extern Draw_Decoration draw_decoration;

extern double average; //used to control the radius of dots

//These are defined in main.h
extern c_dude dude;

//-----------------------------------------------------------------------------

//desired page width 600
const int PS_page_width=600;
float PS_page_scale=1; //calculated based on box width

inline float X(float x){return (x-draw_decoration.box[0])*PS_page_scale;}
inline float Y(float y){return (y-draw_decoration.box[2])*PS_page_scale;}
inline float convert(float v){return v*PS_page_scale;}
inline Point2d convert(const Point2d& p) {  return Point2d(X(p[0]),Y(p[1])); }

void draw_PS_ply(PSDoc *ps, const c_ply& ply)
{
	ply_vertex * ptr=ply.getHead();
    Point2d pos=convert(ptr->getPos());
	PS_moveto(ps,pos[0],pos[1]);

	do{
		const Point2d& pos=convert(ptr->getPos());
		PS_lineto(ps,pos[0],pos[1]);
		ptr=ptr->getNext();
	}while(ptr!=ply.getHead());
	
	PS_closepath(ps);
	PS_fill_stroke(ps);
}

void draw_PS_ply(PSDoc *ps, const c_polygon& polygon)
{
	for(c_polygon::const_iterator i=polygon.begin();i!=polygon.end();i++)
	{
		if(i->getType()==c_ply::POUT)
		{
			draw_PS_ply(ps,*i);
		}
		else{
			PS_setcolor(ps, "fill", "rgb", 1,1,1,0.0);
			PS_setcolor(ps, "stroke", "gray", 0.0, 0.0, 0.0, 0.0);
			draw_PS_ply(ps,*i);
		}
	}
}
void draw_PS_PolygonPieces(PSDoc* ps, const vector<c_polygon*>& polygonPieces)
{
	vector<c_polygon*>::const_iterator pit = polygonPieces.begin();
	for (;pit!=polygonPieces.end();++pit)
	{
		//const c_polygon* & tmpPolygon = *pit;

		for(c_polygon::const_iterator i=(*pit)->begin();i!=(*pit)->end();i++)
		{
			float r1 = rand()*1.0/RAND_MAX;
			float b1 = rand()*1.0/RAND_MAX;
			float g1 = rand()*1.0/RAND_MAX;
			PS_setcolor(ps, "fill", "rgb", r1,g1,b1,0.0);
			if(i->getType()==c_ply::POUT)
			{
				draw_PS_ply(ps,*i);
			}
			else{
				PS_setcolor(ps, "fill", "rgb", 1,1,1,0.0);
				PS_setcolor(ps, "stroke", "gray", 0.0, 0.0, 0.0, 0.0);
				draw_PS_ply(ps,*i);
			}
		}
	}
}

void draw_PS_line(PSDoc *ps, const Point2d& s, const Point2d& t)
{
    Point2d p1=convert(s);
    Point2d p2=convert(t);
    PS_moveto(ps,p1[0],p1[1]);
    PS_lineto(ps,p2[0],p2[1]);
    PS_stroke(ps);
}


void draw_PS_circle(PSDoc *ps, const Point2d& o, float r)
{
    Point2d s1=convert(o);
    PS_circle(ps,s1[0],s1[1],r);
    PS_fill_stroke(ps);
}

void draw_PS_Diagonals(PSDoc *ps, const vector<c_diagonal>& diagonals)
{
    typedef vector<c_diagonal> DIAS;
    for(DIAS::const_iterator i=diagonals.begin();i!=diagonals.end();i++)
        draw_PS_line(ps,i->getV1()->getPos(),i->getV2()->getPos());
}

void draw_PS_Skeleton(PSDoc* ps, const SE2d& se2d)
{
	//cerr<<"print PS skeleton...."<<endl;
	typedef list<se_m*>::const_iterator MIT;
	for(MIT im = se2d.getTodoList().begin();im!=se2d.getTodoList().end();im++){

		vector<pair<pair<sk_node,sk_node>,sk_edge > > edges ;
		int esize = (*im)->skeleton.GetEdgesVData(edges);

		for(int ie = 0; ie != esize; ie++){
			Point2d& s = edges[ie].first.first.m_pos;
			Point2d& t = edges[ie].first.second.m_pos;

			//cerr<<s<<t<<endl;
	        //draw final cuts
	        PS_setlinewidth(ps,4);
	        PS_setcolor(ps, "stroke", "rgb",0,0,0.5,0);
	        draw_PS_line(ps,s,t);
	        PS_setlinewidth(ps,1);

		}

	}


}

inline void draw_PS_Bridge(PSDoc *ps, c_BPC *bpc)
{
    //draw bridge
    draw_PS_line(ps,bpc->getSource1()->getPos().get(),bpc->getSource2()->getPos().get());

    //draw two sources as dots
    PS_setcolor(ps, "fill", "rgb", 0,0,0,0.0);
    PS_setcolor(ps, "stroke", "gray", 0.0, 0.0, 0.0, 0.0);
    draw_PS_circle(ps,bpc->getSource1()->getPos(),average/2);
    draw_PS_circle(ps,bpc->getSource2()->getPos(),average/2);

    //draw kids bridge
    const list<c_BPC *>& kids=bpc->getKids();
    for(list<c_BPC *>::const_iterator i=kids.begin();i!=kids.end();i++){
        c_BPC * kid=*i;
        draw_PS_Bridge(ps,kid);
    }
}

void draw_PS_BPC(PSDoc *ps, c_BPC * bpc, float largest_concavity, float smallest_concavity)
{

    float scale_avg=average/3;

    //draw bridges between sources
    if(draw_options.showBridges)
    {
        //draw bridge
        draw_PS_Bridge(ps,bpc);
    }

    return;

    //draw concavity
    if(draw_options.showConcavity && !draw_options.showPolygonPieceForPS && !draw_options.showSkeletonForPS)
    {
        PS_setcolor(ps, "fill", "rgb", 1,0,0,0.0);
        PS_setcolor(ps, "stroke", "gray", 0.0, 0.0, 0.0, 0.0);
        const list<ply_vertex *>& pms=bpc->getConcavities();
        for(list<ply_vertex *>::const_iterator i=pms.begin();i!=pms.end();i++)
            draw_PS_circle(ps,(*i)->getPos(),scale_avg);

        const list<ply_vertex *>& holefp=dude.holeFeatPnts;
        for(list<ply_vertex *>::const_iterator i=holefp.begin();i!=holefp.end();i++)
        {
            if( (*i)->getExtra().isPM()==false || (*i)->getExtra().diagonals.empty() ) continue;
            draw_PS_circle(ps,(*i)->getPos(),scale_avg);
        }
    }
}

void draw_PS_BPCs(PSDoc *ps)
{
    c_dude& ap=dude;
    const list<c_BPC *>& bpcs=dude.getBPS();
    float pR=getP().front().getRadius();

    typedef list<c_BPC *>::const_iterator IT;
    static float smallest_concavity=FLT_MAX;
    static float largest_concavity=-FLT_MAX;
    if(smallest_concavity==FLT_MAX)
    {
        for(IT i=bpcs.begin();i!=bpcs.end();i++){
            const list<ply_vertex *>& pms=(*i)->getConcavities();
            for(list<ply_vertex *>::const_iterator j=pms.begin();j!=pms.end();j++){
                float c=(*j)->getExtra().concavity;
                if(c<smallest_concavity){
                    smallest_concavity=c;
                }
                if(c>largest_concavity){
                    largest_concavity=c;
                }
            }
        }

        largest_concavity/=pR;
        smallest_concavity/=pR;
    }

    if(draw_options.showConstraints){
        PS_setcolor(ps, "fill", "rgb", 0.7, 0.7, 0.7, 0.0);
        PS_setcolor(ps, "stroke", "gray", 0.0, 0.0, 0.0, 0.0);
        draw_PS_ply(ps,ap.getSimplifiedPolygon());
    }

    if(draw_options.showCuts){

        //draw potential cuts
        PS_setcolor(ps, "stroke", "rgb", 0.5f,0.5f,0.5f, 0.0);
        draw_PS_Diagonals(ps,draw_decoration.allAccumulatedCuts);
        //draw_PS_Diagonals(ps,ap.getDiagonals());

        //draw cut sets
        PS_setlinewidth(ps,2);
        PS_setcolor(ps, "stroke", "rgb", 0.5f,0.85f,0.85f, 0.0);
        {
            c_polygon& P=getP();
            uint Psize=P.getSize();
            for(uint i=0;i<Psize;i++){
                ply_vertex * v=P[i];
                if(v->getExtra().cutsets.empty()) continue;
                vector<c_cutset>& csets=v->getExtra().cutsets;
                uint cset_size=csets.size();
                for(uint j=0;j<cset_size;j++)
                    draw_PS_Diagonals(ps,csets[j].getDiagonals());
            }
        }
        PS_setlinewidth(ps,1);
    }

    if(draw_options.showDecompose)
    {
        //draw final cuts
        PS_setlinewidth(ps,2);
        PS_setcolor(ps, "stroke", "rgb", 0.75,0,0, 0.0);
        //draw_PS_Diagonals(ps,ap.getFinalCuts());
        draw_PS_Diagonals(ps, draw_decoration.allAccumulatedCuts);
        PS_setlinewidth(ps,1);
        PS_setcolor(ps, "stroke", "rgb", 0,0,0, 0.0);
    }

    for(IT i=bpcs.begin();i!=bpcs.end();i++)
        draw_PS_BPC(ps,*i,largest_concavity,smallest_concavity);

}

void drawALL(PSDoc *ps)
{
    float radius = convert(getP().front().getRadius());
    average = 0.02*radius;

    if(draw_options.showP){
        PS_setcolor(ps, "fill", "rgb", 0.9, 0.9, 0.9, 0.0);
        PS_setcolor(ps, "stroke", "gray", 0.0, 0.0, 0.0, 0.0);
        draw_PS_ply(ps,getP());
    }

    draw_PS_BPCs(ps);

	if(draw_options.showPolygonPieceForPS)
		draw_PS_PolygonPieces(ps,draw_decoration.resultPolygons);

	if(draw_options.visulizeSegment)
    {
		PS_setlinewidth(ps,2);
		PS_setcolor(ps, "stroke", "rgb", 1,0,0, 0.0);
		draw_PS_Diagonals(ps, draw_decoration.compDiags);
		PS_setcolor(ps, "stroke", "rgb", 0,0,0, 0.0);
		PS_setlinewidth(ps,1);
    }


	if(draw_options.showSkeletonForPS)
		draw_PS_Skeleton(ps,draw_decoration.se);


	return;

    if(draw_options.showConcavity && !draw_options.showPolygonPieceForPS && !draw_options.showSkeletonForPS)
    {
    	 float scale_avg=average/3;

        PS_setcolor(ps, "fill", "rgb", 0,0,0,0.0);
        PS_setcolor(ps, "stroke", "gray", 0.0, 0.0, 0.0, 0.0);
        const vector<ply_vertex *>& pms=draw_decoration.allFeaturePMs;
        for(vector<ply_vertex *>::const_iterator i=pms.begin();i!=pms.end();i++)
            draw_PS_circle(ps,(*i)->getPos(),scale_avg);

//        const list<ply_vertex *>& holefp=dude.holeFeatPnts;
//        for(list<ply_vertex *>::const_iterator i=holefp.begin();i!=holefp.end();i++)
//        {
//            if( (*i)->getExtra().isPM()==false || (*i)->getExtra().diagonals.empty() ) continue;
//            draw_PS_circle(ps,(*i)->getPos(),scale_avg);
//        }
    }
}


void save2PS(const string& name) 
{
	PS_page_scale=PS_page_width*1.0f/(draw_decoration.box[1]-draw_decoration.box[0]);
	PSDoc *ps=NULL;
	PS_boot();
	ps = PS_new();


	if ( 0 > PS_open_file(ps,name.c_str()) )
	{
		cerr<<"! ERROR: Cannot open PostScript file: "<<name<<endl;
		return;
	}

	PS_set_parameter(ps, "warning", "true");

	PS_set_info(ps, "Creator", __FILE__);
	PS_set_info(ps, "Author", "MASC group@George Mason University");
	PS_set_info(ps, "Title", "dude-Decomposition");
	PS_set_info(ps, "Keywords", "dude-Decomposition");


	PS_begin_page(ps, X(draw_decoration.box[1]), Y(draw_decoration.box[3]));

	drawALL(ps);
	
	PS_end_page(ps);

	PS_close(ps);
	PS_delete(ps);
	PS_shutdown();
}

#endif
