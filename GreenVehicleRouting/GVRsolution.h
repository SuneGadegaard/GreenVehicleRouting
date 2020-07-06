#pragma once
#include<vector>
#include<ilcplex\ilocplex.h>
#include<string>
#include"GVRdata.h"


typedef IloArray<IloNumVarArray> IloVarMatrix;  //!< An IloArray of IloNumVarArrays

class GVRsolution
{
private:
	struct batteryFlow
	{
		int from;
		int to;
		int batteryConsumption;
	};
	int time;
	int maxBatteryConsumption;
	std::vector<std::pair<int, int>> x;
	std::vector<batteryFlow> f;
	std::vector<int> startAtNode;
	std::vector<int> tour;
public:
	GVRsolution ( );
	void setSolution ( IloCplex& cplex, IloVarMatrix& xSol, IloVarMatrix& fSol, IloNumVarArray& tauSol,  int time, int batteryCapacity );
	void printSolution ( );
	void makeTour ( GVRdata* data, std::string& tourFileName );
	void printToTikZFormat ( GVRdata* data, std::string& outFileName );
	int getTime ( ) { return time; }
};

