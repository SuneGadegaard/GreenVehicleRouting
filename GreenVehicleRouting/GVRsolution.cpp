#include"GVRsolution.h"


GVRsolution::GVRsolution ( )
{
}

void GVRsolution::setSolution ( IloCplex& cplex, IloVarMatrix & xSol, IloVarMatrix & fSol, int theTime, int batteryCapacity )
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
		OutFile << "%% Total Time of solution : " << time << " max battery consumption : " << maxBatteryConsumption << "\n";
		OutFile << "\\documentclass[12pt,a4paper]{standalone}\n\\usepackage[utf8]{inputenc}\n\\usepackage{tikz}\n\\usetikzlibrary{shapes.geometric,arrows}\n\\begin{document}\n\t\\begin{tikzpicture}[x = 8pt, y = 8pt]\n\t\t\\tikzstyle{arrow} = [thick, ->, >= stealth, line width = 1.5pt]\n";
		
		// Write out the coordinates in TikZ language
		int numOfNodes = data->getNumOfNodes ( );
		int numOfCust = data->getNumOfCustomers ( );
		for ( int i = 0; i < numOfNodes; ++i )
		{
			int xCoord = data->getCoordinates ( i ).first;
			int yCoord = data->getCoordinates ( i ).second;
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

		int numOfNodes = data->getNumOfNodes ( );
		int numOfCust = data->getNumOfCustomers ( );

		std::vector<std::vector<bool>> incidenceMatrix ( numOfNodes );

		for ( size_t i = 0; i < numOfNodes; ++i )
		{
			incidenceMatrix[i] = std::vector<bool> ( numOfNodes, false );
		}

		for ( auto arc : x )
		{
			incidenceMatrix[arc.first][arc.second] = true;
		}

		int curNode = 0, numOfCustVisited = 0;

		std::vector<int> order;
		std::vector<int> nodesVisited ( numOfNodes, false );
		nodesVisited[0] = true;
		order.push_back ( 0 );
		
		while ( numOfCustVisited < numOfCust )
		{
			for ( int j = 1; j < numOfNodes; ++j )
			{
				if ( incidenceMatrix[curNode][j] )
				{
					if ( j <= numOfCust && !nodesVisited[j] )
					{
						curNode = j;
						order.push_back ( j );
						nodesVisited[j] = true;
						++numOfCustVisited;
						break;
					}
					else if ( j >= numOfCust + 1 )
					{
						curNode = j;
						order.push_back ( j );
						nodesVisited[j] = true;
						break;
					}
				}
			}
		}

		while ( curNode != 0 )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				if ( incidenceMatrix[curNode][j] )
				{
					curNode = j;
					order.push_back ( j );
					nodesVisited[j] = true;
					break;
				}
			}
		}

		OutFile << "Optimal tour is given by : \n";
		for ( auto node = order.begin ( ); node != order.end ( ); ++node )
		{
			if ( std::next ( node ) == order.end ( ) )
			{
				OutFile << *node << "\n";
			}
			else
			{
				OutFile << *node << "->";
			}
		}
		OutFile << "\n\nBattery consumption on each arc (at arrival):\n";
		OutFile << "From\tTo\tBattery consumption\n";
		for ( auto arc : f )
		{
			OutFile << arc.from << "\t" << arc.to << "\t" << arc.batteryConsumption << "\n";
		}
	}
	catch ( const std::exception& e)
	{
		std::cerr << "Exception in makeTour function : " << e.what ( ) << std::endl;
	}
}





