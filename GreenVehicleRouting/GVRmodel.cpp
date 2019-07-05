#include"GVRmodel.h"

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



GVRmodel::GVRmodel ( ) : model ( env ), cplex ( model ), x ( env ), f ( env ), g ( env ), tau ( env ), data ( nullptr )
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
				obj += ( data->getTime ( i, j ) + data->getServiceTime ( i ) )*x[i][j];

			}
			// Remove diagonal as well
			x[i][i].setUB ( 0 );
			f[i][i].setUB ( 0 );
			g[i][i].setUB ( 0 );
		}
		model.add ( IloMinimize ( env, obj ) );
		obj.end ( );
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
			for ( int j = 0; j < numOfNodes; ++j )
			{
				if ( i<= numOfCust && j<=numOfCust) model.add ( f[i][j] <= ( batteryCap - nearestCharger[j - 1]  )* x[i][j] );
				else model.add ( f[i][j] <= batteryCap * x[i][j] );
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

		IloExpr gFlow ( env );
		for ( int i = 0; i < numOfNodes; ++i )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				gFlow += g[i][j] - g[j][i];
				int cap = int ( ( 1.0/2.0 )*double ( numOfCust ) + 0.5 );
				if ( i==0 ) model.add ( g[i][j] <= cap * x[i][j] );
				else model.add ( g[i][j] <= ( cap - 1 )*x[i][j] );
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

void GVRmodel::addTimeVariables ( )
{
	int numOfCust = data->getNumOfCustomers ( );
	int numOfNodes = data->getNumOfNodes ( );
	int tmax = 1500;
	tau = IloNumVarArray ( env, numOfCust + 1, 0, tmax, ILOFLOAT );
	for ( int i = 0; i <= numOfCust; ++i )
	{
		if ( 0 == i )
		{
			for ( int j = 1; j <= numOfCust; ++j )
			{
				if ( i != j ) model.add ( tau[j] >= data->getTime ( 0, j )*x[0][j] );
			}
		}
		else
		{
			for ( int j = 0; j <= numOfCust; ++j )
			{
				if ( i != j ) model.add ( tau[i] - tau[j] + ( data->getServiceTime ( i ) + data->getTime ( i, j ) + tmax )*x[i][j] <= tmax );
			}
		}
	}


	// Here goes the charging part
	for ( int k = numOfCust+1; k <= numOfNodes; ++k )
	{
		for ( int i = 0; i = numOfCust; ++i )
		{
			for ( int j = 0; j <= numOfCust; ++j )
			{
				int tijk = data->getTime ( i, k ) + data->getServiceTime ( i ) + data->getServiceTime ( k ) + data->getTime ( k, i );
				if ( 0 == i ) model.add ( -tau[j] + ( tijk + tmax )*( x[i][k] + x[k][j] ) <= 2 * tmax + tijk );
				else model.add ( tau[i]-tau[j] + ( tijk + tmax )*( x[i][k] + x[k][j] ) <= 2 * tmax + tijk );
			}
		}
	}
}


void GVRmodel::generateBatteryFront ( )
{
	try
	{
		int numOfNodes = data->getNumOfNodes ( );
		
		std::vector<GVRsolution> aFront;
		while ( cplex.solve ( ) )
		{
			int maxBatterryConsumption = 0;
			for ( int i = 0; i < numOfNodes; ++i )
			{
				for ( int j = 0; j < numOfNodes; ++j )
				{
					int fValue = cplex.getValue ( f[i][j] );
					if ( fValue > maxBatterryConsumption ) maxBatterryConsumption = fValue;
				}
			}
			GVRsolution solution = GVRsolution ( );
			solution.setSolution ( cplex, x, f, cplex.getObjValue ( ), maxBatterryConsumption );
			aFront.push_back ( solution );
			for ( int i = 0; i < numOfNodes; ++i )
			{
				for ( int j = 0; j < numOfNodes; ++j )
				{
					f[i][j].setUB ( maxBatterryConsumption - 1 );
				}
			}
			std::cout << "Max battery consumption : " << maxBatterryConsumption << std::endl;
		}

		int solNr = 1;
		std::cout << "Creating the front files\n";
		for ( auto sol : aFront )
		{
			std::string outFile ( "texSolutionFile" );
			std::string outTourFile ( "tourSolutionFile" );
			outFile += std::to_string ( solNr );
			outTourFile += std::to_string ( solNr );
			outFile +=  ".tex";
			outTourFile += ".txt";

			if ( !data->isDataFromMatrixFile ( ) ) sol.printToTikZFormat ( data, outFile );
			sol.makeTour ( data, outTourFile );
			sol.printSolution ( );
			++solNr;	
		}
	}
	catch ( IloException& ie)
	{
		std::cerr << "IloException in generateBatteryFront : " << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}

void GVRmodel::solveModel ( std::string& texFileOutName, std::string& tourFileOutName )
{
	cplex.solve ( );
	GVRsolution solution = GVRsolution ( );
	solution.setSolution ( cplex, x, f, cplex.getObjValue ( ), f[1][2].getUB ( ) );
	
	
	if ( !data->isDataFromMatrixFile ( ) ) solution.printToTikZFormat ( data, texFileOutName );
	
	solution.makeTour ( data , tourFileOutName );
}

void GVRmodel::addTimeWindows ( std::vector<std::pair<int, int>> windows )
{
	IloExtractableArray extractables;

	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfNodes = data->getNumOfNodes ( );
		IloVarMatrix z = IloVarMatrix ( env, numOfCustomers + 1 );
		tau = IloNumVarArray ( env, numOfCustomers, 0, IloInfinity, ILOFLOAT );
		for ( int i = 0; i < numOfCustomers + 1; ++i )
		{
			z[i] = IloNumVarArray ( env, numOfCustomers + 1, 0, 1, ILOBOOL );
		}
		
		for ( int i = 0; i < numOfCustomers + 1; ++i )
		{
			for ( int j = 0; j < numOfCustomers + 1; ++j )
			{
				extractables.add ( model.add ( z[i][j] >= x[i][j] ) );
			}
		}

		IloExpr sumOverK ( env );
		for ( int i = 0; i < numOfCustomers + 1; ++i )
		{
			for ( int j = 0; j < numOfCustomers + 1; ++j )
			{
				for ( int k = numOfCustomers + 1; k < numOfNodes; ++k )
				{
					sumOverK += x[i][k] + x[k][j];
				}
				extractables.add ( model.add ( 2 * z[i][j] <= 2 * x[i][j] + sumOverK ) );
				sumOverK.clear ( );
			}
		}
		sumOverK.end ( );
		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			extractables.add ( model.add ( tau[i] >= data->getTime ( 0, i ) ) );
			extractables.add ( model.add ( tau[i] + data->getTime ( i, 0 )*x[i][0] <= windows[0].second ) );
			tau[i].setBounds ( windows[i].first, windows[i].second );
		}
		tau[0].setBounds ( windows[0].first, windows[0].second );

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
