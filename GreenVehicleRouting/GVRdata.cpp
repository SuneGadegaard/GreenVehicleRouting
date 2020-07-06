#include"GVRdata.h"	

GVRdata::GVRdata ( ) :
	noCapacityRestriction ( true ),
	numOfCustomers ( 0 ),
	numOfChargers ( 0 ),
	numOfNodes ( 0 ),
	numOfVehicles ( 0 ),
	batteryCap ( 0 ),
	batteryConstant ( 1.0 ),
	maxDuration ( 10000000 ),
	vehicleCapacity ( 0 ),
	dataIsFromMatrixFile ( false )
{
}

bool GVRdata::readDataFile ( std::string & file )
{
	std::ifstream InFile;
	std::stringstream err;

	try
	{
		InFile.open ( file );
		if ( !InFile )
		{
			err << "Cannot open the data file\t :\t" << file << std::endl;
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfCustomers;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of customers\n" );
		if ( numOfCustomers <= 1 )
		{
			err << "The number of customers read was below 1. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfChargers;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of charging stations\n" );
		if ( numOfCustomers <= 1 )
		{
			err << "The number of chargers read was below 1. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> batteryCap;
		if ( !InFile ) throw std::runtime_error ( "Could not read the battery capacity\n" );
		if ( batteryCap <= 0 )
		{
			err << "The battery capacity read was below 0. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfVehicles;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of vehicles\n" );
		if ( numOfVehicles <= 0 )
		{
			err << "The number of vehicles read was below 0. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		numOfNodes = 1 + numOfCustomers + numOfChargers;

		serviceTime = std::vector<int> ( numOfNodes );

		for ( int i = 0; i < numOfNodes; ++i )
		{
			InFile >> serviceTime[i];
			if ( !InFile ) throw std::runtime_error ( "Could not read the service times\n" );
			if ( serviceTime[i] < 0 )
			{
				err << "The service time read for node " << i << " was negative. The program terminates\n";
				throw std::invalid_argument ( err.str ( ) );
			}
		}

		coordinates = std::vector<std::pair<double, double>> ( numOfNodes );
		for ( int i = 0; i < numOfNodes; ++i )
		{
			InFile >> coordinates[i].first;
			if ( !InFile ) throw std::runtime_error ( "Could not read the coordinates\n" );
			InFile >> coordinates[i].second;
			if ( !InFile ) throw std::runtime_error ( "Could not read the coordinates\n" );
		}

		generateDistanceMatrix ( );


		// Default the capacity restriction
		if ( noCapacityRestriction )
		{
			vehicleCapacity = numOfNodes;
			for ( size_t i = 0; i < numOfNodes; ++i )
			{
				if ( i == 0 ) demand.push_back ( 0 );
				else if ( i <= numOfCustomers ) demand.push_back ( 1 );
				else demand.push_back ( 0 );
			}
			demand[0] = numOfCustomers;
		}


		return true;
	}
	catch ( const std::exception& e )
	{
		std::cerr << e.what ( ) << std::endl;
		return false;
	}
}

bool GVRdata::readCondensedFile ( std::string & file )
{
	std::ifstream InFile;
	std::stringstream err;
	try
	{
		InFile.open ( file );
		if ( !InFile )
		{
			err << "Cannot open the data file\t :\t" << file << std::endl;
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfCustomers;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of customers\n" );
		if ( numOfCustomers <= 1 )
		{
			err << "The number of customers read was below 1. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfChargers;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of charging stations\n" );
		if ( numOfCustomers <= 1 )
		{
			err << "The number of chargers read was below 1. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> batteryCap;
		if ( !InFile ) throw std::runtime_error ( "Could not read the battery capacity\n" );
		if ( batteryCap <= 0 )
		{
			err << "The battery capacity read was below 0. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfVehicles;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of vehicles\n" );
		if ( numOfVehicles <= 0 )
		{
			err << "The number of vehicles read was below 0. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		numOfNodes = 1 + numOfCustomers + numOfChargers;

		int serviceTimeCustomer = 0;
		InFile >> serviceTimeCustomer;
		if ( !InFile ) throw std::runtime_error ( "Could not read the service time for customers\n" );

		serviceTime = std::vector<int> ( numOfNodes );
		serviceTime[0] = 0;
		for ( int i = 1; i <= numOfCustomers; ++i )
		{
			serviceTime[i] = serviceTimeCustomer;
		}

		int serviceTimeCharger = 0;
		InFile >> serviceTimeCharger;
		if ( !InFile ) throw std::runtime_error ( "Could not read the service time for chargers\n" );
		for ( int i = numOfCustomers + 1; i < numOfNodes; ++i )
		{
			serviceTime[i] = serviceTimeCharger;
		}

		// read the speed of the vehicle
		int speed = 0;
		InFile >> speed;
		if ( !InFile ) throw std::runtime_error ( "Could not read the speed of the vehicle\n" );


		coordinates = std::vector<std::pair<double, double>> ( numOfNodes );
		for ( int i = 0; i < numOfNodes; ++i )
		{
			InFile >> coordinates[i].first;
			if ( !InFile ) throw std::runtime_error ( "Could not read the coordinates\n" );
			InFile >> coordinates[i].second;
			if ( !InFile ) throw std::runtime_error ( "Could not read the coordinates\n" );
		}

		generateDistanceMatrix ( );

		// Overwrite the the time matrix so it is based on the vehicle speed
		double speedFactor = 60.0 / double ( speed );
		for ( int i = 0; i < numOfNodes; ++i )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				time[i][j] = int ( speedFactor * double ( distance[i][j] ) + 0.5 );
			}
		}


		// Default the capacity restriction
		if ( noCapacityRestriction )
		{
			vehicleCapacity = numOfNodes;
			for ( size_t i = 0; i < numOfNodes; ++i )
			{
				if ( i == 0 ) demand.push_back ( 0 );
				else if ( i <= numOfCustomers ) demand.push_back ( 1 );
				else demand.push_back ( 0 );
			}
			demand[0] = numOfCustomers;
		}


		return true;
	}
	catch ( const std::exception& e )
	{
		std::cerr << e.what ( ) << std::endl;
		return false;
	}
}



bool GVRdata::generateDistanceMatrix ( )
{
	try
	{
		distance = std::vector<std::vector<int>> ( numOfNodes );
		time = std::vector<std::vector<int>> ( numOfNodes );
		batteryC = std::vector<std::vector<int>> ( numOfNodes );
		double dist = 0;
		for ( int i = 0; i < numOfNodes; ++i )
		{
			distance[i] = std::vector<int> ( numOfNodes );
			time[i] = std::vector<int> ( numOfNodes );
			batteryC[i] = std::vector<int> ( numOfNodes );

			for ( int j = 0; j < numOfNodes; ++j )
			{
				dist = std::pow ( double ( coordinates[i].first - coordinates[j].first ), 2.0 );
				dist += std::pow ( double ( coordinates[i].second - coordinates[j].second ), 2 );
				distance[i][j] =int ( sqrt ( dist ) + 0.5 );


				time[i][j] = round ( ( 2.0 / 3.0 )*double ( distance[i][j] ) );
				batteryC[i][j] = round ( batteryConstant * double ( distance[i][j] ) );
			}
		}
		return true;
	}
	catch ( const std::exception& e )
	{
		std::cerr << e.what ( ) << std::endl;
		return false;
	}
}

bool GVRdata::readMatrixDataFile ( std::string & file )
{

	std::ifstream InFile;
	std::stringstream err;
	dataIsFromMatrixFile = true;
	try
	{
		InFile.open ( file );
		if ( !InFile )
		{
			err << "Cannot open the data file\t :\t" << file << std::endl;
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfCustomers;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of customers\n" );
		if ( numOfCustomers <= 1 )
		{
			err << "The number of customers read was below 1. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfChargers;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of charging stations\n" );
		if ( numOfCustomers <= 1 )
		{
			err << "The number of chargers read was below 1. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> batteryCap;
		if ( !InFile ) throw std::runtime_error ( "Could not read the battery capacity\n" );
		if ( batteryCap <= 0 )
		{
			err << "The battery capacity read was below 0. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		InFile >> numOfVehicles;
		if ( !InFile ) throw std::runtime_error ( "Could not read the number of vehicles\n" );
		if ( numOfVehicles <= 0 )
		{
			err << "The number of vehicles read was below 0. The program terminates\n";
			throw std::invalid_argument ( err.str ( ) );
		}

		numOfNodes = 1 + numOfCustomers + numOfChargers;

		serviceTime = std::vector<int> ( numOfNodes );

		for ( int i = 0; i < numOfNodes; ++i )
		{
			InFile >> serviceTime[i];
			if ( !InFile ) throw std::runtime_error ( "Could not read the service times\n" );
			if ( serviceTime[i] < 0 )
			{
				err << "The service time read for node " << i << " was negative. The program terminates\n";
				throw std::invalid_argument ( err.str ( ) );
			}
		}

		std::cout << "Got this far\n";

		// Read the distance and time matrices
		distance = std::vector<std::vector<int>> ( numOfNodes );
		time = std::vector<std::vector<int>> ( numOfNodes );
		batteryC = std::vector<std::vector<int>> ( numOfNodes );
		int anInt = 0;
		for ( int i = 0; i < numOfNodes; ++i )
		{
			distance[i] = std::vector<int>(numOfNodes);
			time[i] = std::vector<int> ( numOfNodes );
			batteryC[i] = std::vector<int> ( numOfNodes );
			for ( int j = 0; j < numOfNodes; ++j )
			{
				InFile >> anInt;
				if ( !InFile ) throw std::runtime_error ( "Could not read the distance matrix\n" );
				if ( anInt < 0 )
				{
					err << "The distance between node " << i << " and " << j << " was negative. The program terminates\n";
					throw std::invalid_argument ( err.str ( ) );
				}
				distance[i][j] = int ( double(anInt) * 0.5 );
				batteryC[i][j] = round ( batteryConstant * double ( distance[i][j] ) );
			}
		}

		std::cout << "Distance (38,38) " << distance[37][38] << std::endl;
		std::cout << "Done with the distances\n";
		for ( int i = 0; i < numOfNodes; ++i )
		{
			for ( int j = 0; j < numOfNodes; ++j )
			{
				InFile >> anInt;
				if ( !InFile ) throw std::runtime_error ( "Could not read the time matrix\n" );
				if ( anInt < 0 )
				{
					err << "The time between node " << i << " and " << j << " was negative. The program terminates\n";
					throw std::invalid_argument ( err.str ( ) );
				}
				time[i][j] = int ( double ( anInt ) * 0.5 );
			}
		}


		std::cout << "Done with the times\n";
		// Default the capacity restriction
		if ( noCapacityRestriction )
		{
			vehicleCapacity = numOfNodes;
			for ( size_t i = 0; i < numOfNodes; ++i )
			{
				if ( i == 0 ) demand.push_back ( 0 );
				else if ( i <= numOfCustomers ) demand.push_back ( 1 );
				else demand.push_back ( 0 );
			}
			demand[0] = numOfCustomers;
		}
		std::cout << "Now the capacities are set\n";
//		vehicleCapacity = 8;

		return true;
	}
	catch ( const std::exception& e)
	{
		std::cerr << "Error in the readMatrixDataFile : " << e.what ( ) << std::endl;
		return false;
	}
}

void GVRdata::readAddressesFromFile ( std::string & addressFile )
{
	std::ifstream InFile;
	std::stringstream err;

	try
	{
		InFile.open ( addressFile );
		if ( !InFile )
		{
			err << "Cannot open the data file\t :\t" << addressFile << std::endl;
			throw std::invalid_argument ( err.str ( ) );
		}

		if ( InFile.is_open ( ) ) {
			std::string line;
			while ( getline ( InFile, line ) ) {
				// using printf() in all tests for consistency
				addresses.push_back ( line );
			}
			InFile.close ( );
		}

		for ( auto & line : addresses )
		{
			std::cout << line << "\n";
		}
	}
	catch ( const std::exception& e )
	{
		std::cerr << "Error in the readAddressesFromFile : " << e.what ( ) << std::endl;
	}
}


void GVRdata::setWindowsAndDuration ( std::vector<std::pair<int, int>> newWindows )
{
	timeWindows = newWindows;
	maxDuration = timeWindows[0].second;
}