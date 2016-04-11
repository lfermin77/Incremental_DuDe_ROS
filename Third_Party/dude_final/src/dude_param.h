/*
 * dude_param.h
 *
 *  Created on: Jan 26, 2015
 *      Author: guilin
 */

#ifndef DUDE_PARAM_H_
#define DUDE_PARAM_H_

#include <string>

using namespace std;

struct Dude_Param
{
	Dude_Param()
	{
		rotate_angle = 0;
		concavity_tau = 0.05f;
		decomposeIteratively = true;
		export_decomp_polys = false;
		export_decomp_polys_svg = false;
	}

	string Pfile;
	float rotate_angle;
	float concavity_tau;
	int decomposeIteratively;
	bool export_decomp_polys;
	bool export_decomp_polys_svg;
};



#endif /* DUDE_PARAM_H_ */
