#include"GVRdata.h"
#include"GVRmodel.h"

#include "GVRPKoc.h"

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
			dataReadCorretctly = data->readCondensedFile ( file );
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

		std::vector<std::pair<int, int>> windows;
		for ( int i = 0; i <= data->getNumOfCustomers ( ); ++i )
		{
			windows.push_back ( std::pair<int,int> ( 0, 100000 ) );
		}

		model.setUpProblem ( );
		std::cout << "Before adding time windows\n";
		model.addTimeWindows ( windows );
		std::cout << "After adding time windows\n";
		//model.generateBatteryFront ( );
		std::string texFile ( "texOutFile.tex" );
		std::string tourFile ( "tourOutFile.txt" );
		//model.solveModel ( texFile, tourFile );
		std::cout << "Generating the battery front\n";
		//model.solveModel(texFile, tourFile );
		model.generateBatteryFront ( );




		delete data;
		return true;
	}
	catch ( const std::exception& e)
	{
		std::cerr << "Error : " << e.what ( ) << std::endl;
		return false;
	}
}