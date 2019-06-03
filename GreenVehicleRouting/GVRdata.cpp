#include"GVRdata.h"	

GVRdata::GVRdata ( ) :
	noCapacityRestriction ( true ),
	numOfCustomers ( 0 ),
	numOfChargers ( 0 ),
	numOfNodes ( 0 ),
	numOfVehicles ( 0 ),
	batteryCap ( 0 ),
	batteryConstant ( 1.0 ),
	vehicleCapacity ( 0 )
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

		coordinates = std::vector<std::pair<int, int>> ( numOfNodes );
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

bool GVRdata::generateDistanceMatrix ( )
{
	try
	{
		distance = std::vector<std::vector<int>> ( numOfNodes );
		time = std::vector<std::vector<int>> ( numOfNodes );
		batteryC = std::vector<std::vector<int>> ( numOfNodes );

		for ( int i = 0; i < numOfNodes; ++i )
		{
			distance[i] = std::vector<int> ( numOfNodes );
			time[i] = std::vector<int> ( numOfNodes );
			batteryC[i] = std::vector<int> ( numOfNodes );

			for ( int j = 0; j < numOfNodes; ++j )
			{
				distance[i][j] = std::pow ( double ( coordinates[i].first - coordinates[j].first ), 2.0 );
				distance[i][j] += std::pow ( double ( coordinates[i].second - coordinates[j].second ), 2 );
				distance[i][j] =int ( sqrt ( double(distance[i][j]) ) + 0.5 );


				time[i][j] = round ( ( 2.0 / 3.0 )*double ( distance[i][j] ) ) + serviceTime[i];
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