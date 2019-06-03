#pragma once
#include<vector>
#include<iostream>
#include<fstream>
#include<stdexcept>
#include<string>
#include<sstream>
#include<math.h>

class GVRdata
{
private:
	bool noCapacityRestriction;

	int numOfCustomers;
	int numOfChargers;
	int numOfNodes;
	int numOfVehicles;
	int batteryCap;
	double batteryConstant;

	std::vector<std::pair<int,int>> coordinates;
	std::vector<int> serviceTime;
	std::vector<std::vector<int>> distance;
	std::vector<std::vector<int>> time;
	std::vector<std::vector<int>> batteryC;
	std::vector<int> demand;
	int vehicleCapacity;

	bool generateDistanceMatrix ( );

public:
	GVRdata ( );
	bool readDataFile ( std::string &file );
	int getDist ( int i, int j ) { return distance[i][j]; }
	int getTime ( int i, int j ) { return time[i][j]; }
	int getBatteryConsumption ( int i, int j ) { return batteryC[i][j]; }
	int getServiceTime ( int i ) { return serviceTime[i]; }
	int getNumOfCustomers ( ) { return numOfCustomers; }
	int getNumOfChargers ( ) { return numOfChargers; }
	int getNumOfNodes ( ) { return numOfNodes; }
	int getNumOfVehicles ( ) { return numOfVehicles; }
	int getBatteryCap ( ) { return batteryCap; }
	int getDemands ( int i ) { return demand[i]; }
	int getVehicleCapacity ( ) { return vehicleCapacity; }
	std::pair<int, int> getCoordinates ( int i ) { return coordinates[i]; }
};
