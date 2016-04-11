#ifndef _SK2D_SKELETON_H_
#define _SK2D_SKELETON_H_

#include <string>
#include <iostream>
#include <stdlib.h>
using namespace std;

class se_m;

class ExtracSkeleton
{
public:
	virtual void extract(se_m * m)=0;
};

///////////////////////////////////////////////////////////////////////////////
class ES_PA : public ExtracSkeleton
{
	virtual void extract(se_m * m);
};

///////////////////////////////////////////////////////////////////////////////
class ES_Centers : public ExtracSkeleton
{
	virtual void extract(se_m * m);
};

///////////////////////////////////////////////////////////////////////////////
class ES_Factory{
public:
	static ExtracSkeleton * create_ES(const string& name ){
		if(name=="PA" || name=="Principle Axis")
			return new ES_PA();
		else if (name=="Centers" || name=="centers" )
			return new ES_Centers();
		else{
			cerr<<"! ERROR: ES_Factory::create_ES: Unknow type="<<
				name<<endl;
			exit(1);
		}
	}
};

#endif //_SK2D_SKELETON_H_
