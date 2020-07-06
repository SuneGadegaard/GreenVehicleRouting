#include"GVRmodel.h"
#include<string>
#include<string.h>
#include<algorithm>

ILOUSERCUTCALLBACK2 ( cutter, IloVarMatrix, x, GVRdata*, data )
{
	try
	{

	}
	catch ( IloException & ie )
	{
		std::cerr << "IloException in cut-callback : " << ie.getMessage ( ) << std::endl;
	}
}



GVRmodel::GVRmodel ( ) : model ( env ), cplex ( model ), x ( env ), f ( env ), g ( env ), tau ( env ), z(env), data ( nullptr )
{
}

GVRmodel::~GVRmodel ( )
{
	for ( auto itt : batteryUBs )
	{
		for ( auto it : itt )
		{
			if ( it.getImpl ( ) ) it.end ( );
		}
	}
	if ( g.getImpl ( ) )
	{
		for ( size_t i = 0; i < g.getSize ( ); ++i )
		{
			if ( g[i].getImpl ( ) )
			{
				g[i].end ( );
			}
		}
	}

	if ( f.getImpl ( ) )
	{
		for ( size_t i = 0; i < g.getSize ( ); ++i )
		{
			if ( g[i].getImpl ( ) )
			{
				g[i].end ( );
			}
		}
	}

	if ( x.getImpl ( ) )
	{
		for ( size_t i = 0; i < g.getSize ( ); ++i )
		{
			if ( g[i].getImpl ( ) )
			{
				g[i].end ( );
			}
		}
	}

	if ( tau.getImpl ( ) )
	{
		tau.end ( );
	}

	if ( z.getImpl ( ) )
	{
		for ( size_t i = 0; i < z.getSize ( ); ++i )
		{
			if ( z[i].getImpl ( ) )
			{
				z[i].end ( );
			}
		}
	}

	for ( auto itt : batteryUBs )
	{
		for ( auto it : itt )
		{
			if ( it.getImpl ( ) )
			{
				it.end ( );
			}
		}
	}

	if ( cplex.getImpl ( ) ) cplex.end ( );
	if ( model.getImpl ( ) ) model.end ( );
	if ( env.getImpl ( ) ) env.end ( );
}

bool GVRmodel::setUpProblem ( )
{
	try
	{
		if ( data == nullptr ) throw std::runtime_error ( "The data object was not correctly set. It is still a null pointer\n" );

		int numCust = data->getNumOfCustomers ( );
		int numCharge = data->getNumOfChargers ( );
		int numVehicles = data->getNumOfVehicles ( );
		int numOfNodes = data->getNumOfNodes ( );

		x = IloVarMatrix ( env, numOfNodes );
		f = IloVarMatrix ( env, numOfNodes );
		g = IloVarMatrix ( env, numOfNodes );
		
		addObjectiveFunction ( );
		addDegreesConstraintsOfCustomersX ( );
		addDepotDegreeX ( );
		addDegreeConstraintsOfChargersX ( );
		addBoundsOnF ( );
		addBatteryFlowConstraints ( );
		addConnectivity ( );
	}
	catch ( const std::exception &e )
	{
		std::cerr << "Exception in setUpProblem : " << e.what ( ) << std::endl;
		return false;
	}
}

void GVRmodel::addObjectiveFunction ( )
{
	try
	{
		IloExpr obj ( env );
		int numOfNodes = data->getNumOfNodes ( );
		for ( int i = 0; i < numOfNodes; ++i )
		{
			x[i] = IloNumVarArray ( env, numOfNodes, 0, 1, ILOBOOL );
			f[i] = IloNumVarArray ( env, numOfNodes, 0, data->getBatteryCap(), ILOFLOAT );
			g[i] = IloNumVarArray ( env, numOfNodes, 0, data->getVehicleCapacity(), ILOFLOAT );
			for ( int j = 0; j < numOfNodes; ++j )
			{
				std::string stringName ( "x_" + std::to_string ( i ) + "_" + std::to_string ( j ) );
				char charName[100];
				strcpy_s ( charName, stringName.c_str ( ) );
				x[i][j].setName ( charName );

				stringName  = std::string( "f_" + std::to_string ( i ) + "_" + std::to_string ( j ) );
				strcpy_s ( charName, stringName.c_str ( ) );
				f[i][j].setName ( charName );

				stringName = std::string ( "g_" + std::to_string ( i ) + "_" + std::to_string ( j ) );
				strcpy_s ( charName, stringName.c_str ( ) );
				g[i][j].setName ( charName );
			};
			for ( int j = 0; j < numOfNodes; ++j )
			{
				obj += int ( double ( double ( data->getTime ( i, j ) ) + double ( data->getServiceTime ( i ) ) ) )*x[i][j];
			}
			// Remove diagonal as well
			x[i][i].setUB ( 0 );
			f[i][i].setUB ( 0 );
			g[i][i].setUB ( 0 );
		}
		model.add ( IloMinimize ( env, obj ) );
		obj.end ( );

		// Eliminate all connections between two chargers:
		int numOfCustomers = data->getNumOfCustomers ( );
		for ( int i = numOfCustomers + 1; i < numOfNodes; ++i )
		{
			x[0][i].setUB ( 0 );
			x[i][0].setUB ( 0 );
			for ( int j = numOfCustomers + 1; j < numOfNodes; ++j )
			{
				x[i][j].setUB ( 0 );
			}
		}
	}
	catch ( IloException& ie )
	{
		std::cerr << "IloException in addObjectiveFunction : " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}

}

void GVRmodel::addDegreesConstraintsOfCustomersX ( )
{
	try
	{
		IloExpr inDegX ( env ), outDegX ( env );
		int numOfCust = data->getNumOfCustomers ( );
		int numOfNodes = data->getNumOfNodes ( );
		for ( int i = 1; i <= numOfCust; ++i )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				inDegX += x[j][i];
				outDegX += x[i][j];
			}
			model.add ( inDegX == 1 );
			model.add ( outDegX == 1 );
			inDegX.clear ( );
			outDegX.clear ( );
		}
		inDegX.end ( );
		outDegX.end ( );
	}
	catch ( IloException&  ie )
	{
		std::cerr << "IloException in addDegreesConstraintsOfCustomersX : " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

void GVRmodel::addDepotDegreeX ( )
{
	try
	{
		IloExpr in ( env ), out ( env );
		int numOfNodes = data->getNumOfNodes ( );
		for ( int i = 0; i < numOfNodes; ++i )
		{
			in += x[i][0];
			out += x[0][i];
		}
		model.add ( in == data->getNumOfVehicles ( ) );
		model.add ( out == data->getNumOfVehicles ( ) );
		in.end ( );
		out.end ( );
	}
	catch ( IloException& ie )
	{
		std::cerr << "IloException in addDepotDegreeX : " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

void GVRmodel::addDegreeConstraintsOfChargersX ( )
{
	try
	{
		IloExpr cst ( env );
		int numOfNodes = data->getNumOfNodes ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfCust = data->getNumOfChargers ( );

		for ( int i = numOfCust + 1; i < numOfNodes; ++i )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				cst += x[i][j] - x[j][i];
			}
			model.add ( cst == 0 );
			cst.clear ( );
		}

		cst.end ( );
	}
	catch ( IloException& ie )
	{
		std::cerr << "IloException in void addDegreeConstraintsOfChargersX : " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

void GVRmodel::addBoundsOnF ( )
{
	try
	{
		int numOfNodes = data->getNumOfNodes ( );
		int numOfCust = data->getNumOfCustomers ( );
		int batteryCap = data->getBatteryCap ( );

		//Start by finding the nearest charging station for each customer
		std::vector<int> nearestCharger ( numOfCust );
		for ( int i = 1; i <= numOfCust; ++i )
		{
			nearestCharger[i - 1] = data->getBatteryConsumption ( i, 0 );
			for ( int j = numOfCust + 1; j < numOfNodes; ++j )
			{
				int batteryConsumption = data->getBatteryConsumption ( i, j );
				if ( batteryConsumption < nearestCharger[i - 1] )
				{
					nearestCharger[i - 1] = batteryConsumption;
				}
			}
		}


		for ( int i = 0; i < numOfNodes; ++i )
		{
			batteryUBs.push_back ( std::vector<IloRange> ( numOfNodes ) );
			for ( int j = 0; j < numOfNodes; ++j )
			{
				IloRange bound;
				
				if ( i <= numOfCust && j <= numOfCust ) 
				{
					bound = IloRange ( env, -IloInfinity, f[i][j] - ( batteryCap - nearestCharger[j - 1] )* x[i][j] , 0 );

					//model.add ( f[i][j] <= ( batteryCap - nearestCharger[j - 1] )* x[i][j] );
				}
				else
				{
					bound = IloRange ( env, -IloInfinity, f[i][j] - batteryCap * x[i][j], 0 );
					//model.add ( f[i][j] <= batteryCap * x[i][j] );
				}
				batteryUBs[i][j] = bound;
				model.add ( bound );
				model.add ( f[i][j] >= data->getBatteryConsumption ( i, j )*x[i][j] );
			}
		}

		for ( int i = numOfCust + 1; i < numOfNodes; ++i )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				model.add ( f[i][j] == data->getBatteryConsumption ( i, j )*x[i][j] );
			}
		}

		for ( int j = 0; j < numOfNodes; ++j )
		{
			model.add ( f[0][j] == data->getBatteryConsumption ( 0, j )*x[0][j] );
		}

	}
	catch ( IloException& ie )
	{
		std::cerr << "IloException in addBoundsOnF : " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

void GVRmodel::addBatteryFlowConstraints ( )
{
	try
	{
		int numOfNodes = data->getNumOfNodes ( );
		int numOfCust = data->getNumOfCustomers ( );
		int batteryCap = data->getBatteryCap ( );

		IloExpr fFlowOut ( env ), fFlowIn (env), xConsum(env);
		for ( int i = 1; i <= numOfCust; ++i )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				fFlowOut += f[i][j];
				fFlowIn += f[j][i];
				xConsum += data->getBatteryConsumption ( i, j )*x[i][j];
			}
			model.add ( fFlowOut == fFlowIn + xConsum );
			fFlowOut.clear ( );
			fFlowIn.clear ( );
			xConsum.clear ( );
		}
		fFlowOut.end ( );
		fFlowIn.end ( );
		xConsum.end ( );

	}
	catch ( IloException& ie )
	{
		std::cerr << "IloException in addBatteryFlowConstraints: " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

void GVRmodel::addConnectivity ( )
{
	try
	{
		int numOfNodes = data->getNumOfNodes ( );
		int numOfCust = data->getNumOfCustomers ( );
		int batteryCap = data->getBatteryCap ( );
		int cap = data->getVehicleCapacity();

		IloExpr gFlow ( env );
		for ( int i = 0; i < numOfNodes; ++i )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				gFlow += g[i][j] - g[j][i];
				
				if ( i==0 ) model.add ( g[i][j] <= cap * x[i][j] );
				else model.add ( g[i][j] <=  cap * x[i][j] );
			}
			if ( 0 != i ) model.add ( gFlow == -data->getDemands ( i ) );
			else model.add ( gFlow == data->getDemands ( i ) );

			gFlow.clear ( );
		}
		gFlow.end ( );

	}
	catch ( IloException& ie )
	{
		std::cerr << "IloException in addConnectivity: " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

//=====================================================================================//
void GVRmodel::fixVariables ( )
{
	int numOfCustomers = data->getNumOfCustomers ( );
	int numOfNodes = data->getNumOfNodes ( );
	int maxDuration = data->getMaxDuration ( );
	int numOfFixed = 0;
	// Check whether we have t_0i+t_ij+t_j0+p_i+p_j>t^max
	// Also check if t_0i+t_ij+p_i> min ( b_j , maxDuration - t_j0)
	for ( int i = 1; i <= numOfCustomers; ++i )
	{
		for ( int j = 1; j <= numOfCustomers; ++j )
		{
			if ( i != j ) 
			{
				int t_0i = data->getTime ( 0, i);
				int t_ij = data->getTime ( i, j );
				int t_j0 = data->getTime ( j, 0 );
				int p_i = data->getServiceTime ( i );
				int p_j = data->getServiceTime ( j );
				
				if ( t_0i + t_ij + t_j0 + p_i + p_j > maxDuration )
				{
					x[i][j].setUB ( 0 );
					++numOfFixed;
				}
				else if ( t_0i + t_ij + p_i > std::min ( data->getLateStart ( j ), maxDuration - t_j0 ) )
				{
					x[i][j].setUB ( 0 );
					++numOfFixed;
				}
			}
		}
	}

	std::cout << "Number of fixed variables : " << numOfFixed << std::endl;

}

//=====================================================================================//

void GVRmodel::generateBatteryFront ( )
{
	try
	{
		/**
		Set a starting solution
		**/
		IloNumVarArray xvars ( env );
		IloNumArray xvals ( env );
		int numOfCustomers = data->getNumOfCustomers ( );

		cplex.setParam ( IloCplex::EpAGap, 0.0 );
		cplex.setParam ( IloCplex::EpGap, 0.0 );

		for ( int i = 0; i <= numOfCustomers; ++i )
		{
			if ( i < numOfCustomers ) xvars.add ( x[i][i + 1] );
			else xvars.add ( x[i][0] );
			xvals.add ( 1 );
		}
		cplex.addMIPStart ( xvars, xvals, IloCplex::MIPStartEffort::MIPStartSolveFixed );

		int numOfNodes = data->getNumOfNodes ( );
		int solNr = 1;
		std::vector<GVRsolution> aFront;

		cplex.setParam ( IloCplex::TiLim, 3600 );

		while ( cplex.solve ( ) )
		{
			
			int maxBatterryConsumption = 0;
			
			int counter = 0;
			for ( int i = 0; i < numOfNodes; ++i )
			{
				for ( int j = 0; j < numOfNodes; ++j )
				{
					int fValue = cplex.getValue ( f[i][j] );
					if ( fValue > maxBatterryConsumption ) maxBatterryConsumption = fValue;
					//batteryUBs[i][j].setLinearCoef ( x[i][j], maxBatterryConsumption - 1 );
				}
			}
			GVRsolution solution = GVRsolution ( );
			solution.setSolution ( cplex, x, f, tau, cplex.getObjValue ( ), maxBatterryConsumption );
			aFront.push_back ( solution );
			std::cout << "Max battery consumption : " << maxBatterryConsumption << std::endl;





			std::string outFile ( "texSolutionFile" );
			std::string outTourFile ( "tourSolutionFile" );
			outFile += std::to_string ( solNr );
			outTourFile += std::to_string ( solNr );
			outFile += ".tex";
			outTourFile += ".txt";

			std::cout << "Creating the tour file: " << outTourFile << "\n";

			if ( !data->isDataFromMatrixFile ( ) ) solution.printToTikZFormat ( data, outFile );
			solution.makeTour ( data, outTourFile );
			solution.printSolution ( );
			++solNr;

			//cplex.setParam ( IloCplex::CutUp, 481 );

			int time = aFront.back ( ).getTime ( );
			if ( time > 480 )
			{
				break;
			}

			for ( int i = 0; i < numOfNodes; ++i )
			{
				for ( int j = 0; j < numOfNodes; ++j )
				{
					f[i][j].setUB ( maxBatterryConsumption - 1 );
					if ( data->getBatteryConsumption ( i, j ) >= maxBatterryConsumption - 1 ) x[i][j].setUB ( 0 );
				}
			}
			cplex.resetTime ( );
		}


			
	}
	catch ( IloException& ie)
	{
		std::cerr << "IloException in generateBatteryFront : " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

//=====================================================================================//
void GVRmodel::solveModel ( std::string& texFileOutName, std::string& tourFileOutName )
{
	cplex.exportModel ( "Model.lp" );
	fixVariables ( );
	cplex.setParam ( IloCplex::MIPSearch, 1 );
	cplex.setParam ( IloCplex::Param::MIP::Tolerances::MIPGap, 0.1 );
	if ( cplex.solve ( ) )
	{

		std::cout << "========================================================\n";
		std::cout << "=========== RESTARTING THE OPTIMIZATION ================\n";
		std::cout << "========================================================\n";
		model.add ( x[1][2] + x[2][1] <= 1 );
		cplex.setParam ( IloCplex::MIPSearch, 0 );
		cplex.setParam ( IloCplex::Param::MIP::Tolerances::MIPGap, 0.0 );
		cplex.setParam ( IloCplex::Param::MIP::Tolerances::AbsMIPGap, 0.999 );

		cplex.solve ( );
		std::cout << "============= best obj value : " << cplex.getObjValue ( ) << "=============\n";
		GVRsolution solution = GVRsolution ( );
		solution.setSolution ( cplex, x, f, tau, cplex.getObjValue ( ), f[1][2].getUB ( ) );

		if ( !data->isDataFromMatrixFile ( ) ) solution.printToTikZFormat ( data, texFileOutName );

		solution.makeTour ( data, tourFileOutName );
	}
	else
	{
		std::cout << "============= Did not solve model =============\n";
	}

	
}

//=====================================================================================//
void GVRmodel::addTimeWindows ( std::vector<std::pair<int, int>> windows )
{
	IloExtractableArray extractables (env);

	try
	{
		data->setWindowsAndDuration ( windows );

		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfNodes = data->getNumOfNodes ( );
		z = IloVarMatrix ( env, numOfCustomers + 1 );
		tau = IloNumVarArray ( env, numOfCustomers + 1, 0, IloInfinity, ILOFLOAT );
	
		for ( int i = 0; i <= numOfCustomers; ++i )
		{
			std::string stringName ( "tau_" + std::to_string ( i ) );
			char charName[100];
			strcpy_s ( charName, stringName.c_str ( ) );
			tau[i].setName ( charName );
		}

		for ( int i = 0; i < numOfCustomers + 1; ++i )
		{
			z[i] = IloNumVarArray ( env, numOfCustomers + 1, 0, 1, ILOBOOL);
			
			for ( int j = 0; j <= numOfCustomers; ++j )
			{
				std::string stringName ( "z_" + std::to_string ( i ) + "_" + std::to_string ( j ) );
				char charName[100];
				strcpy_s ( charName, stringName.c_str ( ) );
				z[i][j].setName ( charName );
			}
		}
		model.add ( tau[0] == 0 );
		std::cout << "First stop\n";
		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			for ( int j = 1; j <= numOfCustomers; ++j )
			{
				extractables.add ( model.add ( z[i][j] >= x[i][j] ) );
			}
		}


		std::cout << "Second stop\n";
		IloExpr sumOverK ( env );
		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			for ( int j = 1; j <= numOfCustomers; ++j )
			{
				for ( int k = numOfCustomers + 1; k < numOfNodes; ++k )
				{
					sumOverK += ( x[i][k] + x[k][j] );
				}
				extractables.add ( model.add ( 2 * z[i][j] <= 2 * x[i][j] + sumOverK ) );
				//model.add ( 2 * z[i][j] <= 2 * x[i][j] + sumOverK );
				sumOverK.clear ( );
			}
		}
		std::cout << "Third stop\n";
		sumOverK.end ( );
		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			// Cannot arrive at customer i earlier than if we go directly from the depot
			extractables.add ( model.add ( tau[i] >= data->getTime ( 0, i ) ) );
			// The time we arrive at customer i cannot be later than we can return to the depot again
			extractables.add ( model.add ( tau[i] + ( data->getTime ( i, 0 ) + data->getServiceTime ( i ) )*x[i][0] <= windows[0].second ) );
			int timeFromDepot = data->getTime ( 0, i );
			int timeToDepot = data->getTime ( i, 0 );

			tau[i].setBounds ( std::max( windows[i].first , timeFromDepot ) , std::min ( windows[i].second, windows[0].second - timeToDepot) );
		}
		tau[0].setBounds ( windows[0].first, windows[0].second );

		std::cout << "Fourth stop\n";
		// Add time accumulation constraints
		IloExpr zSum ( env ), zSumRev ( env );
		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			z[i][i].setBounds ( 0, 0 );
			std::cout << "Fifth stop i=" <<i <<"\n" ;
			for ( int j = 1; j <= numOfCustomers; ++j )
			{
				if ( i != j )
				{
					extractables.add ( model.add ( tau[j] >= tau[i] + ( data->getTime ( i, j ) + data->getServiceTime ( i ) )*x[i][j] - windows[0].second*( 1 - x[i][j] ) ) );

					for ( int k = numOfCustomers + 1; k < numOfNodes; ++k )
					{
						int fromItoK = data->getTime ( i, k ) + data->getServiceTime ( i );
						int fromKtoJ = data->getTime ( k, j ) + data->getServiceTime ( k );
						int bigM = windows[0].second - data->getTime ( i, 0 ) - windows[j].first + data->getTime ( i, k ) + data->getTime ( k, j );
						extractables.add ( model.add ( tau[j] >= tau[i] + fromItoK * x[i][k] + fromKtoJ * x[k][j] - bigM * ( 1 - z[i][j] ) ) );
					}
				}
				zSum += z[i][j];
				zSumRev += z[j][i];
			}
			extractables.add ( model.add ( zSum + x[i][0] == 1 ) );
			extractables.add ( model.add ( zSumRev + x[0][i] == 1 ) );
			zSum.clear ( );
			zSumRev.clear ( );
		}
		
		zSum.end ( );
		zSumRev.end ( );

	

	}
	catch ( const std::runtime_error& re )
	{
		std::cerr << "Runtime error in addTimeWindows : " << re.what ( ) << "\n";
		cleanupTimeWindows( extractables );
		return;
	}
	catch ( const std::exception& e )
	{
		std::cerr << "Runtime error in addTimeWindows : " << e.what ( ) << "\n";
		cleanupTimeWindows ( extractables );
		return;
	}
	catch ( ... )
	{
		std::cerr << "Unknown exception caught in addTimeWindows. No time windows are added";
		cleanupTimeWindows ( extractables );
		return;
	}
}

void GVRmodel::cleanupTimeWindows ( IloExtractableArray& extractables )
{
	model.remove ( extractables );
}
