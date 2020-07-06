#pragma once
#include<ilcplex\ilocplex.h>
#include<vector>
#include"GVRdata.h"

typedef IloArray<IloNumVarArray> IloVarMatrix;

class GVRPKoc
{
private:

	IloEnv env;
	IloModel model;
	IloCplex cplex;

	IloVarMatrix x;
	std::vector<std::vector<std::vector<IloNumVar>>> y;
	IloNumVarArray f;
	IloNumVarArray tau;

	std::vector<std::vector<std::vector<int>>> chargeDist;
	std::vector<std::vector<std::vector<int>>> chargeTime;
	void makeChargeDistsAndTime ( );
	GVRdata *data;

	void add17 ( );
	void add18 ( );
	void add19 ( );
	void add20 ( );
	void add21 ( );
	void add22 ( );
	void add23 ( );
	void add24 ( );
	void add25 ( );

	void addNames ( );
public:
	GVRPKoc ( );
	~GVRPKoc ( );
	void setData ( GVRdata* theData ) { if ( theData == nullptr ) return; data = theData; makeChargeDistsAndTime ( ); }
	void setUpModel ( );
};
