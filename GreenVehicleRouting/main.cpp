#include"GVRdata.h"
#include"GVRmodel.h"
#include<iostream>
int main ( int argc, char* argv[])
{

	try
	{
		GVRdata* data = new GVRdata ( );
		std::string file ( argv[1] );
		bool dataReadCorretctly;

		if ( true )
		{
			// Use the matrix file format
			std::cout << "In the right bracket\n";
			dataReadCorretctly = data->readMatrixDataFile ( file );
		}
		else
		{
			dataReadCorretctly = data->readMatrixDataFile ( file );
		}

		if ( !dataReadCorretctly )
		{
			throw std::exception ( "Could not read data file!\n" );
		}





		GVRmodel model = GVRmodel ( );
		model.setData ( data );

		model.setUpProblem ( );
		model.generateBatteryFront ( );
		/*std::string texFile ( "texOutFile.tex" );
		std::string tourFile ( "tourOutFile.txt" );
		model.solveModel ( texFile, tourFile );*/

		delete data;
		return true;
	}
	catch ( const std::exception& e)
	{
		std::cerr << "Error : " << e.what ( ) << std::endl;
		return false;
	}
}