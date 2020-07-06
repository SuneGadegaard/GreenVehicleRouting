#include"GVRPKoc.h"
#include<algorithm>
#include<string>



GVRPKoc::GVRPKoc ( ) : model ( env ), cplex ( model ), x ( env ), f ( env ), tau ( env )
{
}

GVRPKoc::~GVRPKoc ( )
{
	if ( f.getImpl ( ) ) f.end ( );
	if ( tau.getImpl ( ) ) tau.end ( );
	if ( x.getImpl ( ) )
	{
		for ( int i = 0; i < x.getSize ( ); ++i )
		{
			if ( x[i].getImpl ( ) ) x[i].end ( );
		}
	}

	for ( auto yi : y )
	{
		for ( auto yij : yi )
		{
			for ( auto yijk : yij )
			{
				if ( yijk.getImpl ( ) ) yijk.end ( );
			}
		}
	}
}


void GVRPKoc::makeChargeDistsAndTime ( )
{
	int numOfCustomers = data->getNumOfCustomers ( );
	int numOfChargers = data->getNumOfChargers ( );
	for ( int i = 0; i <= numOfCustomers; ++i )
	{
		chargeDist.push_back ( std::vector < std::vector<int> >( numOfCustomers + 1 ) );
		chargeTime.push_back ( std::vector < std::vector<int> > ( numOfCustomers + 1 ) );
		for ( int j = 0; j <= numOfCustomers; ++j )
		{
			for ( int k = 0; k < numOfChargers; ++k )
			{
				int chargeIndex = numOfCustomers + 1 + k;
				int dist = data->getDist ( i, chargeIndex ) + data->getDist ( chargeIndex, j );
				int time = data->getTime ( i, chargeIndex ) + data->getTime ( chargeIndex, j ) + data->getServiceTime ( i ) + data->getServiceTime ( chargeIndex );
				chargeDist[i][j].push_back ( dist );
				chargeTime[i][j].push_back ( time );
			}
		}
	}
}


void GVRPKoc::setUpModel ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfNodes = data->getNumOfNodes ( );
		int numOfChargers = data->getNumOfChargers ( );
		int batteryCap = data->getBatteryCap ( );

		x = IloVarMatrix ( env, numOfCustomers + 1 );
		f = IloNumVarArray ( env, numOfCustomers + 1, 0, batteryCap, ILOFLOAT );
		tau = IloNumVarArray ( env, numOfCustomers + 1, 0, data->getMaxDuration ( ), ILOFLOAT );

		IloExpr obj ( env );
		for ( int i = 0; i <= numOfCustomers; ++i )
		{
			x[i] = IloNumVarArray ( env, numOfCustomers + 1, 0, 1, ILOBOOL );
			x[i][i].setUB ( 0 );
			y.push_back ( std::vector < std::vector<IloNumVar> > ( numOfCustomers + 1 ) );
			for ( int j = 0; j <= numOfCustomers; ++j )
			{
				for ( int k = 0; k < numOfChargers; ++k )
				{
					y[i][j].push_back ( IloNumVar ( env, 0, 1, ILOBOOL ) );
					obj += chargeTime[i][j][k] * y[i][j][k];
					if ( i == j ) y[i][j][k].setUB ( 0 );
				}
				
				obj += ( data->getTime ( i, j ) + data->getServiceTime ( i ) )*x[i][j];
			}
		}

		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			for ( int k = 0; k < numOfChargers; ++k )
			{
				y[0][i][k].setUb ( 0 );
				y[i][0][k].setUB ( 0 );
			}
		}

		model.add ( IloMinimize ( env, obj ) );
		
		obj.end ( );

		add17 ( );
		add18 ( );
		add19 ( );
		add20 ( );
		add21 ( );
		add22 ( );
		add23 ( );
		add24 ( );
		add25 ( );
		addNames ( );

		


		IloNumVarArray numThroughCharger = IloNumVarArray ( env, numOfChargers, 0, numOfCustomers, ILOINT );
		for ( int k = 0; k < numOfChargers; ++k )
		{
			std::string stringName ( "num_" + std::to_string ( k ) );
			char charName[100];
			strcpy_s ( charName, stringName.c_str ( ) );
			numThroughCharger[k].setName ( charName );
		}

		IloExpr cst ( env );
		for ( int k = 0; k < numOfChargers; ++k )
		{
			for ( int i = 0; i < numOfCustomers; ++i )
			{
				for ( int j = i + 1; j <= numOfCustomers; ++j )
				{
					cst += y[i][j][k] + y[j][i][k];
				}
			}
			model.add ( cst == numThroughCharger[k] );
			cst.clear ( );
			cplex.setPriority ( numThroughCharger[k], 10 );
		}
		cst.end ( );

	
		for ( int i = 0; i <= numOfCustomers; ++i )
		{
			for ( int j = 0; j <= numOfCustomers; ++j )
			{
				if ( i!=j) cplex.setPriority ( x[i][j], 1 );
			}
		}
		
		
		
		IloNumVarArray vars ( env );
		IloNumArray vals ( env );

		vars.add ( x[0][1] );
		vars.add ( x[1][2] );
		vars.add ( x[2][3] );
		vars.add ( x[3][4] );
		vars.add ( x[4][5] );
		vars.add ( x[5][6] );
		vars.add ( x[6][7] );
		vars.add ( x[7][8] );
		vars.add ( x[8][9] );
		vars.add ( x[9][10] );
		vars.add ( x[10][0] );
		for ( int i = 0; i <= numOfCustomers; ++i )vals.add ( 1 );

		cplex.addMIPStart ( vars, vals, IloCplex::MIPStartSolveFixed );
		std::cout << data->getMaxDuration ( ) <<"===========KAFDNGAÆLKVFBNADÆGBKNADFÆGBNA\n";
		cplex.exportModel ( "kocModel.lp" );
		cplex.setParam ( IloCplex::MIPSearch, 1 );
		cplex.solve ( );

		std::cout << "i\t j\t time\t batteryC\t tau_i\t tau_j\t f_i\t f_j\n";

		for ( int i = 0; i <= numOfCustomers; ++i )
		{
			for ( int j = 0; j <= numOfCustomers; ++j )
			{
				if ( i != j && cplex.getValue ( x[i][j] ) >= 0.5 )
				{
					std::cout << i << "\t"
						<< j << "\t"
						<< data->getTime ( i, j ) << "\t"
						<< data->getBatteryConsumption ( i, j ) << "\t";
					if ( i > 0 ) std::cout << cplex.getValue ( tau[i] ) << "\t";
					else std::cout << "---\t";
					if ( j > 0 ) std::cout << cplex.getValue ( tau[j] ) << "\t";
					else std::cout << "---\t";

					/*if ( i > 0 ) std::cout << cplex.getValue ( f[i] ) << "\t";
					else std::cout << "---\t";
					if ( j > 0 ) std::cout << cplex.getValue ( tau[j] ) << std::endl;
					else std::cout << "---" << std::endl;*/
					std::cout << "\n";
				}
			}
		}
	}
	catch ( const std::exception& e)
	{
		std::cerr << "Exception in setUpModel : " << e.what ( ) << std::endl;
	}
	catch ( IloException &ie )
	{
		std::cerr << ie.getMessage ( ) << std::endl;
		ie.end ( );
	}
}


void GVRPKoc::add17 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );

		IloExpr cst ( env );
		for ( int j = 1; j <= numOfCustomers; ++j )
		{
			for ( int i = 0; i <= numOfCustomers; ++i )
			{
				cst + x[i][j];
				for ( int k = 0; k < numOfChargers; ++k )
				{
					cst += y[i][j][k];
				}
			}
			model.add ( cst == 1 );
			cst.clear ( );
		}
		cst.end ( );
	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add17 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::add18 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );

		IloExpr cst ( env );
		for ( int j = 0; j <= numOfCustomers; ++j )
		{
			for ( int i = 0; i <= numOfCustomers; ++i )
			{
				cst += x[i][j] - x[j][i];
				for ( int k = 0; k < numOfChargers; ++k )
				{
					cst += y[i][j][k] - y[j][i][k];
				}
			}
			model.add ( cst == 0 );
			cst.clear ( );
		}
		cst.end ( );

	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add18 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::add19 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );

		IloExpr cst ( env );
		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			for ( int j = 1; j <= numOfCustomers; ++j )
			{
				for ( int k = 0; k < numOfChargers; ++k )
				{
					int M1ijk = data->getMaxDuration ( ) + data->getTime ( i, j ) + chargeTime[i][j][k] - data->getTime ( i, 0 ) - data->getTime ( 0, j );
					cst += tau[i] - tau[j] + ( M1ijk - chargeTime[i][j][k] )*x[i][j];
					cst += ( M1ijk - chargeTime[i][j][k] - data->getTime ( i, j ) - data->getTime ( j, i ) )*x[j][i];
					cst += ( M1ijk - data->getTime ( i, j ) )*y[i][j][k];
					cst += ( M1ijk - data->getTime ( i, j ) - chargeTime[i][j][k] - chargeTime[j][i][k] )*y[j][i][k];
					model.add ( cst <= M1ijk - data->getTime ( i, j ) - chargeTime[i][j][k] );
					cst.clear ( );
				}
			}
		}
		cst.end ( );
	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add19 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::add20 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );

		IloExpr cst ( env );
		for ( int j = 1; j <= numOfCustomers; ++j )
		{
			for ( int k = 0; k < numOfChargers; ++k )
			{
				cst += chargeTime[0][j][k] * y[0][j][k];
			}
			model.add ( tau[j] >= data->getTime ( 0, j )*x[0][j] + cst );
			cst.clear ( );
		}
		cst.end ( );
	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add20 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::add21 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );

		IloExpr cst ( env );
		for ( int j = 1; j <= numOfCustomers; ++j )
		{
			for ( int k = 0; k < numOfChargers; ++k )
			{
				cst += ( data->getMaxDuration ( ) - chargeTime[0][j][k] )*y[0][j][k];
			}
			model.add ( tau[j] <= data->getMaxDuration ( ) - ( data->getMaxDuration ( ) - data->getTime ( 0, j ) )*x[0][j] - cst );
			cst.clear ( );
		}
		cst.end ( );
	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add21 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::add22 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );

		IloExpr cst ( env );
		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			for ( int k = 0; k < numOfChargers; ++k )
			{
				cst += chargeTime[i][0][k] * y[i][0][k];
			}
			model.add ( tau[i] <= data->getMaxDuration ( ) - data->getTime ( i, 0 )*x[i][0] - cst );
			cst.clear ( );
		}
		cst.end ( );
	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add22 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::add23 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );
	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add23 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::add24 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );
	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add24 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::add25 ( )
{
	try
	{
		int numOfCustomers = data->getNumOfCustomers ( );
		int numOfChargers = data->getNumOfChargers ( );
		int numOfNodes = data->getNumOfNodes ( );
	}
	catch ( const std::exception& e )
	{
		std::cerr << "exception in add25 : " << e.what ( ) << std::endl;
	}
}

void GVRPKoc::addNames ( )
{
	int numOfCustomers = data->getNumOfCustomers ( );
	int numOfChargers = data->getNumOfChargers ( );
	int numOfNodes = data->getNumOfNodes ( );

	for ( int i = 0; i <= numOfCustomers; ++i )
	{
		std::string stringName ( "tau_" + std::to_string ( i ) );
		char charName[100];
		strcpy_s ( charName, stringName.c_str ( ) );
		tau[i].setName ( charName );
	}

	for ( int i = 0; i <= numOfCustomers; ++i )
	{
		std::string stringName ( "f" + std::to_string ( i ) );
		char charName[100];
		strcpy_s ( charName, stringName.c_str ( ) );
		f[i].setName ( charName );
	}

	for ( int i = 0; i <= numOfCustomers; ++i )
	{
		for ( int j = 0; j <= numOfCustomers; ++j )
		{
			std::string stringName ( "x_" + std::to_string ( i ) + "_" + std::to_string ( j ) );
			char charName[100];
			strcpy_s ( charName, stringName.c_str ( ) );
			x[i][j].setName ( charName );
		}
	}

	for ( int i = 0; i <= numOfCustomers; ++i )
	{
		for ( int j = 0; j <= numOfCustomers; ++j )
		{
			for ( int k = 0; k < numOfChargers; ++k )
			{
				std::string stringName ( "y_" + std::to_string ( i ) + "_" + std::to_string ( j ) + "_" + std::to_string ( k ) );
				char charName[100];
				strcpy_s ( charName, stringName.c_str ( ) );
				y[i][j][k].setName ( charName );
			}
		}
	}
}
