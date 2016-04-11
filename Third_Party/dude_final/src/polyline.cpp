
//------------------------------------------------------------------------------
//  Copyright 2007-2012 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#include "polyline.h"

#include <vector>


    ///////////////////////////////////////////////////////////////////////////////
    
    //copy from c_ply
    c_plyline::c_plyline(c_ply& ply)
    {
    	destroy();//detroy myself first
    	ply_vertex* ptr=ply.getHead();
        beginPoly();
        do{
            const Point2d& pos=ptr->getPos();
            addVertex(pos[0],pos[1]);
            ptr=ptr->getNext();
        }while( ptr!=ply.getHead() );
        
        //add head again
        const Point2d& head_pos = ply.getHead()->getPos();
        addVertex(head_pos[0], head_pos[1]);
        endPoly();	
    }	
        
    //copy from the given ply
    void c_plyline::copy(const c_plyline& other)
    {
        destroy();//detroy myself first

        ply_vertex* ptr=other.head;
        beginPoly();
        do{
            ply_vertex * v=new ply_vertex();
            assert(v); //check for memory
            v->copy(ptr);
            addVertex(v);
            ptr=ptr->getNext();
        }while( ptr!=NULL );

        endPoly();
    }

    // clean up the space allocated
    void c_plyline::destroy()
    {
        if( head==NULL ) return;
        ply_vertex* ptr=head;
        do{
            ply_vertex * n=ptr->getNext();
            delete ptr;
            ptr=n;
        }while( ptr!=NULL );
        head=tail=NULL;

        all.clear();
    }

    // Create a empty polygon
    void c_plyline::beginPoly()
    {
        head=tail=NULL;
        all.clear();
    }

    // Add a vertex to the polygonal chian
    void c_plyline::addVertex( double x, double y, bool remove_duplicate )
    {
        Point2d pt(x,y);

        if(tail!=NULL){
            if(tail->getPos()==pt && remove_duplicate) return; //don't add
        }

        ply_vertex * v=new ply_vertex(pt);
        if( tail!=NULL ){
            tail->setNext(v);
        }
        tail=v;
        if( head==NULL ) head=tail;
        v->setVID(all.size()); //id of the vertex in this ply
        all.push_back(v);

    }

    // Add a vertex to the polygonal chian
    void c_plyline::addVertex( ply_vertex * v )
    {
        if( tail!=NULL ){
            tail->setNext(v);
        }
        tail=v;
        if( head==NULL ) head=tail;
        v->setVID(all.size()); //id of the vertex in this ply
        all.push_back(v);
    }

    // finish building the polygon
    void c_plyline::endPoly()
    {        
        doInit();
    }

    // initialize property of the this polychain
    // Compute normals and find reflective vertices
    void c_plyline::doInit()
    {
        //compute normals
        ply_vertex* ptr=head;
        do{
	        ptr->computeExtraInfo();
            ptr=ptr->getNext();
        }while( ptr!=NULL );
    }

    ///////////////////////////////////////////////////////////////////////////
    //convert plyline to ply
    c_ply c_plyline::toply() const
    {
        c_ply ply(c_ply::UNKNOWN);
    	ply_vertex* ptr=getHead();
    	ply.beginPoly();
        do{
            const Point2d& pos=ptr->getPos();
            ply.addVertex(pos[0],pos[1]);
            ptr=ptr->getNext();
        }while( ptr!=NULL );
        ply.endPoly();	
        return ply;
    }
    
    ///////////////////////////////////////////////////////////////////////////
    void c_plyline::negate()
    {
        ply_vertex * ptr=head;
        do{
            ptr->negate();
            ptr=ptr->getNext();
        }while(ptr!=NULL); //end while
    }

    ///////////////////////////////////////////////////////////////////////////
    void c_plyline::translate(const Vector2d& v)
    {
        ply_vertex * ptr=head;
        do{
            ptr->translate(v);
            ptr=ptr->getNext();
        }while(ptr!=NULL); //end while
    }

    void c_plyline::rotate(double radius)
    {
        ply_vertex * ptr=head;
        do{
            ptr->rotate(radius);
            ptr=ptr->getNext();
        }while(ptr!=NULL); //end while
    }

    void c_plyline::expand(c_plyline& plyline)
    {
        //cout<<"my head="<<head<<" my tail="<<tail<<" other head="<<plyline.head<<" other tail="<<plyline.tail<<endl;

        //remove plyline's head if tail and head coincide
        if(tail->getPos()==plyline.head->getPos()){
            ply_vertex * new_head=plyline.head->getNext();
            new_head->setPre(NULL);
            delete plyline.head;
            plyline.head=new_head;
        }


        tail->setNext(plyline.head);
        tail->computeExtraInfo();
        tail=plyline.tail;
        build_all();
    }

    void c_plyline::build_all()
    {
        uint vid=0;
        all.clear();
        ply_vertex * ptr=head;
        do{
            ptr->setVID(vid++);
            all.push_back(ptr);
            ptr=ptr->getNext();
        }while(ptr!=NULL); //end while
    }

    //
    // Compute the center and the box of a list of plys
    //

    //
    // Compute the center and the box of a list of plys
    //
    void c_plylinelist::translate(const Vector2d& v)
    {
        for(iterator i=begin();i!=end();i++) i->translate(v);
    }

    //
    // rotate all polychains
    //
    void c_plylinelist::rotate(double r)
    {
        for(iterator i=begin();i!=end();i++) i->rotate(r);
    }

    void c_plylinelist::negate()
    {
        for(iterator i=begin();i!=end();i++) i->negate();
    }


    //
    // Compute the center and the box of a list of plys
    //

    void c_plylinelist::buildBoxAndCenter()
    {
        //typedef list<c_ply>::iterator IT;
        box[0]=box[2]=FLT_MAX;
        box[1]=box[3]=-FLT_MAX;
        for(iterator i=begin();i!=end();i++){

            ply_vertex * ptr=i->getHead();
            do{
                const Point2d& p=ptr->getPos();
                if(p[0]<box[0]) box[0]=p[0];
                if(p[0]>box[1]) box[1]=p[0];
                if(p[1]<box[2]) box[2]=p[1];
                if(p[1]>box[3]) box[3]=p[1];
                ptr=ptr->getNext();
            }
            while(ptr!=NULL); //end while
        }

        center[0]=(box[0]+box[1])/2;
        center[1]=(box[2]+box[3])/2;

        is_buildboxandcenter_called=true;
    }

    //
	// IO operators
	//
    istream& operator>>( istream& is, c_plyline& poly)
    {
        int vsize;
        is>>vsize;

        poly.beginPoly();
        
        //read in all the vertices
        int iv;
        vector< pair<double,double> > pts; pts.reserve(vsize);
        for( iv=0;iv<vsize;iv++ ){
            double x,y;
            is>>x>>y;
            pts.push_back(pair<double,double>(x,y));
        }

        int id;
        for( iv=0;iv<vsize;iv++ ){
            is>>id; id=id-1;
            poly.addVertex(pts[id].first,pts[id].second);
        }

        poly.endPoly();
        return is;
    }

    istream& operator>>( istream& is, c_plylinelist& p)
    {
        //remove header commnets
        do{
            char tmp[1024];
            char c=is.peek();
            if(isspace(c)) is.get(c); //eat it
            else if(c=='#') {
                is.getline(tmp,1024);
            }
            else break;
        }while(true);

        //start reading
        uint size;
        is>>size;
        uint vid=0;
        for(uint i=0;i<size;i++){
            c_plyline poly;
            is>>poly;
            p.push_back(poly);
            uint vsize=poly.getSize();
            for(uint j=0;j<vsize;j++){
                poly[j]->setVID(vid++);       //id of vertex in the polygons
            }
        }
        
        //done
        return is;
    }

    ostream& operator<<( ostream& os, c_plyline& p)
    {
        os<<p.getSize()<<"\n";
        ply_vertex * ptr=p.head;
        do{
            os<<ptr->getPos()[0]<<" "<<ptr->getPos()[1]<<"\n";
            ptr=ptr->getNext();
        }
        while(ptr!=NULL);

        for(int i=0;i<p.getSize();i++) os<<i+1<<" ";
        os<<"\n";
        return os;
    }

    ostream& operator<<( ostream& out, c_plylinelist& p)
    {
        out<<p.size()<<"\n";
        typedef c_plylinelist::iterator PIT;
        for(PIT i=p.begin();i!=p.end();i++) out<<*i;
        return out;
    }

