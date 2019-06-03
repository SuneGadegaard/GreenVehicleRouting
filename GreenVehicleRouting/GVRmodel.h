#pragma once
#include<ilcplex\ilocplex.h>
#include<vector>
#include"GVRdata.h"
#include"GVRsolution.h"


typedef IloArray<IloNumVarArray> IloVarMatrix;  //!< An IloArray of IloNumVarArrays

class GVRmodel
{
private:
	IloEnv env;			//! IloEnv on which the model and algorithm is build
	IloModel model;		//! IloModel in which the model is managed
	IloCplex cplex;		//! IloCplex used to solve the integer linear program(s)
	IloVarMatrix x;		//! Arc variables. x[i][j]=1 if a vehicle traverses the arc (i,j). x[i][j]=0 otherwise
	IloVarMatrix f;		//! If x[i][j]=0 => f[i][j]=0. Otherwise, f[i][j] is the energy consumed since last charging when arriving at node j
	IloVarMatrix g;		//! If x[i][j]=0 => g[i][j]=0. Otherwise, g[i][j] equals the amount of flow of a commodity flowing on arc (i,j)
	GVRdata* data;		//! Pointer to the GVRdata object holding the data for the green vehicle routing problem.
	std::vector<std::vector< IloRange >> batteryUBs;	//! Vector of vectors of IloRanges used to hold the upper bounds for the battery consumption on an arc. Used in epsilon algorithm

	/*! \brief Adds the objective function 
	 * 
	 * Builds and adds the objective function to the model
	 */
	void addObjectiveFunction ( ); 
	
	/*! \brief Adds the degree constraints for the customers for the x variables
	 * 
	 * Builds and adds the degree constraints for the x variables to the model
	 */
	void addDegreesConstraintsOfCustomersX ( );

	/*! \brief Adds the degree constraints for the x variables for the depot
	*
	* Builds and adds the degree constraints for the x variables for the depot to the model
	*/
	void addDepotDegreeX ( );

	/*! \brief Adds the degree constraints for the x variables for the charging stations
	*
	* Builds and adds the degree constraints for the x variables for the charging stations to the model
	*/
	void addDegreeConstraintsOfChargersX ( );

	/*! \brief Adds the bounds on the f variables
	*
	* Builds and adds the upper bounds for the f variables to the model
	*/
	void addBoundsOnF ( );

	/*! \brief Adds the battery flow constraints
	*
	* Builds and adds the degree battery flow constraints to the model
	*/
	void addBatteryFlowConstraints ( );

	/*! \brief Adds connectivity constraints to the model
	 *
	 * Adds connectivity constraints to the model in the form of a one-commedity flow
	 */
	void addConnectivity ( );

public:
	GVRmodel ( );
	~GVRmodel ( );
	
	/*! \brief Sets the data file 
	 *
	 * Sets the data file for the model to be equal to theData
	 * \param theData pointer to an GVRdata object
	 * \note It is assumed that GVRdata object pointed to contains meaningfull data. No extensive checking is done!
	 */
	void setData ( GVRdata* theData ) { if ( theData == nullptr ) return; data = theData; }

	/*! \brief Build the Green Vehicle Routing problem
	 * 
	 * Builds the Green Vehicle Routing problem as an integer linear programming problem.
	 * \note The function setData(...) should be called before calling setUpProblem. The program throws error otherwise.
	 */
	bool setUpProblem ( );

	/*! \brief Sets a time limit for the optimization
	 *
	 * Sets a time limit in seconds for the optimization algorithm. That is, the limit applies to the optimization only, not for setting up the model.
	 * \param timeInSeconds integer specifying the number of seconds the algorithm should be run. If not set, the time limit defualts to CPLEX' default value.
	 */
	void setTimeLimit ( int timeInSeconds ) { cplex.setParam ( IloCplex::TiLim, timeInSeconds ); }

	/*! \brief Generates a frontier based on max-battery consumption
	 *
	 * Gradually lowers the battery level based on the current solution. Generates all weakly non-dominated solutions wrt. time and battery consumption
	 */
	void generateBatteryFront ( );

	void solveModel ( std::string& texFileOutName, std::string& tourFileOutName  );
};

