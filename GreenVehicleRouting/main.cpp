#include"GVRdata.h"
#include"GVRmodel.h"
#include<iostream>
int main ( int argc, char* argv[])
{

	GVRdata* data = new GVRdata ( );
	std::string file ( argv[1] );
	data->readDataFile ( file );
	




	GVRmodel model = GVRmodel ( );
	model.setData ( data );

	model.setUpProblem ( );
	//model.generateBatteryFront ( );
	std::string texFile ( "texOutFile.tex" );
	std::string tourFile ( "tourOutFile.txt" );
	model.solveModel ( texFile, tourFile );
	
	delete data;
	return true;
}