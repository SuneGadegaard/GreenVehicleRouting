#include"GVRsolution.h"


GVRsolution::GVRsolution ( )
{
}

void GVRsolution::setSolution ( IloCplex& cplex, IloVarMatrix & xSol, IloVarMatrix & fSol, IloNumVarArray& tauSol, int theTime, int batteryCapacity )
{
	try
	{
		time = theTime;
		maxBatteryConsumption = batteryCapacity;
		for ( size_t i = 0; i < xSol.getSize ( ); ++i )
		{
			for ( size_t j = 0; j < xSol[i].getSize ( ); ++j )
			{
				if ( cplex.getValue ( xSol[i][j] ) >= 0.5 )
				{
					x.push_back ( std::pair<int, int> ( i, j ) );
					batteryFlow bf;
					bf.from = i;
					bf.to = j;
					bf.batteryConsumption = cplex.getValue ( fSol[i][j] );
					f.push_back ( bf );
				}
			}
		}
		for ( size_t i = 0; i < tauSol.getSize ( ); ++i )
		{
			startAtNode.push_back ( cplex.getValue ( tauSol[i] ) );
		}
	}
	catch ( const std::exception& e )
	{
		std::cerr << "Exception in saveSolution of the GVRsolution class: " << e.what ( ) << std::endl;
	}
	catch ( IloException &ie )
	{
		std::cerr << "IloException in saveSolution of the GVRsolution class: " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

void GVRsolution::printSolution ( )
{
	std::cout << time << "\t" << maxBatteryConsumption << std::endl;
}



void GVRsolution::printToTikZFormat ( GVRdata* data , std::string& outFileName )
{
	try
	{
		std::ofstream OutFile;
		OutFile.open ( outFileName );
		if ( !OutFile ) std::cout << "SHIT the output file is not open correctly!\n";
		OutFile << "%% Total Time of solution : " << time << " max battery consumption : " << maxBatteryConsumption << "\n";
		OutFile << "\\documentclass[12pt,a4paper]{standalone}\n\\usepackage[utf8]{inputenc}\n\\usepackage{tikz}\n\\usetikzlibrary{shapes.geometric,arrows}\n\\begin{document}\n\t\\begin{tikzpicture}[x = 15pt, y = 15pt]\n\t\t\\tikzstyle{arrow} = [thick, ->, >= stealth, line width = 1.5pt]\n";
		
		// Write out the coordinates in TikZ language
		int numOfNodes = data->getNumOfNodes ( );
		int numOfCust = data->getNumOfCustomers ( );


		for ( int i = 0; i < numOfNodes; ++i )
		{
			double xCoord = data->getCoordinates ( i ).first;
			double yCoord = data->getCoordinates ( i ).second;
			if ( 0 == i )
			{
				OutFile << "\t\t\\node[regular polygon,regular polygon sides=4, draw] (0) at (" << xCoord << "," << yCoord << "){" << i << "};\n";
			}
			else if ( i <= numOfCust )
			{
				OutFile << "\t\t\\node[draw, circle] (" << i << ") at (" << xCoord << "," << yCoord << ") { " << i << " };\n";
			}
			else
			{
				OutFile << "\t\t\\node[regular polygon, regular polygon sides = 3, draw, fill = green] (" << i << ") at (" << xCoord << "," << yCoord << " ) {" << i << "};\n";
			}
		}

		for ( auto & arc : f  )
		{
			OutFile << "\t\t\\path[arrow] (" << arc.from << ") edge node[fill = white, anchor = center, pos = 0.5]{" << arc.batteryConsumption << "} (" << arc.to << ");" << "\n";
		}

		OutFile << "\t\\end{tikzpicture}\n\\end{document}\n";
		OutFile.close ( );

	}
	catch ( const std::exception& e )
	{
		std::cerr << "Exception in printToTikZFormat : " << e.what ( ) << std::endl;
	}
}


void GVRsolution::makeTour ( GVRdata* data, std::string& tourFileName )
{
	try
	{
		std::ofstream OutFile;
		OutFile.open ( tourFileName );
		
		if ( OutFile )
		{
			std::cout << "Tour file is open \n";
		}
		else
		{
			std::cout << "Tour file is not open \n";
		}

		int numOfNodes = data->getNumOfNodes ( );
		int numOfCust = data->getNumOfCustomers ( );

		OutFile << "Optimal time of route : " << time << "\n";
		OutFile << "Largest battery consumption : " << maxBatteryConsumption << "\n";
		OutFile << "From\tTo\tBattery\tTravel-T\tTravel-D\t StartAtToNode\n";
		for ( auto& arc : f )
		{
			std::cout << "writing stuff to the outfile\n";
			OutFile << arc.from << "\t"
				<< arc.to
				<< "\t"
				<< arc.batteryConsumption
				<< "\t"
				<< data->getTime ( arc.from, arc.to )
				<< "\t"
				<< data->getDist ( arc.from, arc.to )
				<< "\t";
			std::cout << "Before the if statements\n";
			if ( arc.to > startAtNode.size ( ) )
			{
				OutFile << "---\n";
			}
			else
			{
				OutFile << startAtNode[arc.to] << "\n";
			}
			std::cout << "After writing to file\n";
		}
		OutFile.close ( );


	}
	catch ( const std::exception& e)
	{
		std::cerr << "Exception in makeTour function : " << e.what ( ) << std::endl;
	}
}





