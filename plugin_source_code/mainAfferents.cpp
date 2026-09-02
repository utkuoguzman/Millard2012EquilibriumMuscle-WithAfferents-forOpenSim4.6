/*  
 *  This program is a forward simulation of a tug-of-war between two muscles 
 *  pulling on a block. One of the muscles has afferents and the other does not.
 *  Both muscles should produce equal forces when given equal stimulation,
 *  initial positions, and parameters.
 *  Modified from the tugOfWar program for the Custom Muscle tutorial. 
 */

//=============================================================================
//=============================================================================
#include <OpenSim/OpenSim.h>
#include "Millard12EqMuscleWithAfferents.h"
#include "SpindleController.h"
#include <OpenSim/Common/IO.h>
#include "AfferentAnalysis.h"
#include "TugOfWarController.h"

using namespace OpenSim;
using namespace SimTK;

//_____________________________________________________________________________
/**
 * Run a simulation of a sliding block being pulled by two muscles 
 */
int main()
{
    std::clock_t startTime = std::clock(); // moved this below

	try {
		///////////////////////////////////////////////
		// DEFINE THE SIMULATION START AND END TIMES //
		///////////////////////////////////////////////
		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 2.0;

		///////////////////////////////////////////
		// DEFINE BODIES AND JOINTS OF THE MODEL //
		///////////////////////////////////////////
		// Create an OpenSim model and set its name
		Model osimModel;
		osimModel.setName("tugOfWar");

		// GROUND BODY

		// Get a reference to the model's ground body
		OpenSim::Body& ground = osimModel.getGroundBody();

		// Add display geometry to the ground to visualize in the GUI
		ground.addDisplayGeometry("ground.vtp");
		ground.addDisplayGeometry("anchor1.vtp");
		ground.addDisplayGeometry("anchor2.vtp");

		// BLOCK BODY

		// Specify properties of a 20(?) kg, 10cm length block body
		double blockMass = 2.0, blockSideLength = 0.1;
		Vec3 blockMassCenter(0);
		blockMassCenter(1) = blockSideLength/2.0;
		Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, 
			blockSideLength, blockSideLength);

		// Create a new block body with the specified properties
		OpenSim::Body *block = new OpenSim::Body("block", blockMass, 
			blockMassCenter, blockInertia);

		// Add display geometry to the block to visualize in the GUI
		block->addDisplayGeometry("block.vtp");

		// FREE JOINT

		// Create a new free joint with 6 degrees-of-freedom (coordinates) 
		// between the block and ground bodies
		double halfLength = blockSideLength/2.0;
		Vec3 locationInParent(0, halfLength, 0), orientationInParent(0);
		Vec3 locationInBody(0, halfLength, 0), orientationInBody(0);
		FreeJoint *blockToGround = new FreeJoint("blockToGround", ground, 
			locationInParent, orientationInParent, 
			*block, locationInBody, orientationInBody);
		
		// Get a reference to the coordinate set (6 degrees-of-freedom) 
		// between the block and ground bodies
		CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();

		// Set the angle and position ranges for the coordinate set
		double angleRange[2] = {-SimTK::Pi/2, SimTK::Pi/2};
		double positionRange[2] = {-1, 1};
		jointCoordinateSet[0].setRange(angleRange);
		jointCoordinateSet[1].setRange(angleRange);
		jointCoordinateSet[2].setRange(angleRange);
		jointCoordinateSet[3].setRange(positionRange);
		jointCoordinateSet[4].setRange(positionRange);
		jointCoordinateSet[5].setRange(positionRange);

		// Add the block body to the model
		osimModel.addBody(block);

		///////////////////////////////////////
		// DEFINE FORCES ACTING ON THE MODEL //
		///////////////////////////////////////
		// MUSCLE FORCES
		// Create two new muscles.
		// Force = [N], length = [m], angle = [rad]

		// Afferent muscle (Millard2012EquilibriumMuscle with afferents)
		double maxIsometricForceAff = 1000.0, optimalFiberLengthAff = 0.109, 
			   tendonSlackLengthAff = 0.071,    pennationAngleAff = 0.0;
		Millard12EqMuscleWithAfferents* Afferent = new Millard12EqMuscleWithAfferents("Afferent",
			maxIsometricForceAff, optimalFiberLengthAff, tendonSlackLengthAff, 
			pennationAngleAff);

		// original muscle model (muscle without afferents)
		double maxIsometricForceOr = 1000.0, optimalFiberLengthOr = 0.2, 
			   tendonSlackLengthOr = 0.1,    pennationAngleOr = 0.0;
		Millard2012EquilibriumMuscle* original = 
			new Millard2012EquilibriumMuscle("original",
				maxIsometricForceOr, optimalFiberLengthOr, tendonSlackLengthOr,
				pennationAngleOr);

		// Define the path of the muscles
		Afferent->addNewPathPoint("Afferent-point1", ground, 
			Vec3(0.0, halfLength, -0.225));
		Afferent->addNewPathPoint("Afferent-point2", *block, 
			Vec3(0.0, halfLength, -halfLength));

		original->addNewPathPoint("original-point1", ground, 
			Vec3(0.0, halfLength, 0.35));
		original->addNewPathPoint("original-point2", *block, 
			Vec3(0.0, halfLength, halfLength));

		// Define the default states for the two muscles
		// Activation
		Afferent->setDefaultActivation(0.01);
		original->setDefaultActivation(0.01);
		// Fiber length
		Afferent->setDefaultFiberLength(optimalFiberLengthAff);
		original->setDefaultFiberLength(optimalFiberLengthOr);

		// Add the two muscles (as forces) to the model
		osimModel.addForce(Afferent);
		osimModel.addForce(original);

		///////////////////////////////////
		// DEFINE CONTROLS FOR THE MODEL //
		///////////////////////////////////

	//-------------------------------------------------------
		// This section uses a TugOfWarController
		// The input functions are set in TugOfWarController.cpp
		double kp = 2200.0;  // position gain for TugOfWarController /1600
		double kv = 500.0;	// velocity gain for TugOfWarController  /80
		TugOfWarController *TOWcontrol = new TugOfWarController(kp,kv);
		TOWcontrol->setActuators( osimModel.updActuators() );
		osimModel.addController( TOWcontrol );
	//-------------------------------------------------------
/*	
	//-------------------------------------------------------
		// Alternatively, this section creates a SpindleController and
		// a PrescribedController.

		// For muscles, controls are normalized motor-neuron excitations
		SpindleController *spinController = new SpindleController();
		PrescribedController *muscleController = new PrescribedController();

		// need to use a Set<Actuator> object as argument to setActuators
		Set<Actuator> *spinActSet = new Set<Actuator>;
		spinActSet->adoptAndAppend(dynamic_cast<Actuator*>(&(osimModel.updActuators().get("Afferent"))));
		spinController->setActuators(*spinActSet);

		Set<Actuator> *muscActSet = new Set<Actuator>;
		muscActSet->adoptAndAppend(dynamic_cast<Actuator*>(&(osimModel.updActuators().get("original"))));
		muscleController->setActuators(*muscActSet);
	
		// Set a control function for the prescribed controllers
		double *Tvalues; Tvalues = new double[4]; 
		double *Cvalues; Cvalues = new double[4];
		Tvalues[0] = 0.2; Tvalues[1] = 0.5; Tvalues[2] = 0.7; Tvalues[3] = 1.0;
		Cvalues[0] = 0.1; Cvalues[1] = 0.9; Cvalues[2] = 0.7; Cvalues[3] = 0.9;
		spinController->prescribeControlForActuator("Afferent", new Constant(0.3),
													new Constant(10.0), new Constant(110.0));
		muscleController->prescribeControlForActuator("original", 
		                                    new PiecewiseLinearFunction(4,Tvalues,Cvalues));

		// Add the muscle controllers to the model
		osimModel.addController(spinController);
		osimModel.addController(muscleController);
	//-------------------------------------------------------
*/

		// Add a Muscle analysis
		MuscleAnalysis* muscAnalysis = new MuscleAnalysis(&osimModel);
		Array<std::string> coords(jointCoordinateSet[5].getName(),1);
		muscAnalysis->setCoordinates(coords);
		muscAnalysis->setComputeMoments(false);
		osimModel.addAnalysis(muscAnalysis);
		
		// Add the analyzer for the muscle with afferents
		AfferentAnalysis* affAnalysis = new AfferentAnalysis(&osimModel);
		affAnalysis->specifyMuscle("Afferent");
		osimModel.addAnalysis(affAnalysis);

		// Turn on the visualizer to view the simulation run live.
		osimModel.setUseVisualizer(false);

		//////////////////////////
		// PERFORM A SIMULATION //
		//////////////////////////

		// Initialize the system and get the state
		SimTK::State& si = osimModel.initSystem();

		// Init coords to 0 and lock the rotational degrees of freedom so the block doesn't twist
		CoordinateSet& coordinates = osimModel.updCoordinateSet();
		coordinates[0].setValue(si, 0);
		coordinates[1].setValue(si, 0);
		coordinates[2].setValue(si, 0);
		coordinates[3].setValue(si, 0);
		coordinates[4].setValue(si, 0); 
		coordinates[5].setValue(si, 0);
		coordinates[0].setLocked(si, true);
		coordinates[1].setLocked(si, true);
		coordinates[2].setLocked(si, true);
		// Last coordinate (index 5) is the Z translation of the block
		coordinates[4].setLocked(si, true); 

		// Get the z translation coordinate.
		Coordinate& zCoord = coordinates.get( "blockToGround_zTranslation" );
		// Set z translation speed value to something congruent
		// with the desired trajectory
		zCoord.setSpeedValue( si, 0.0 * Pi );
		
		// Compute initial conditions for muscles
		osimModel.equilibrateMuscles(si);

		// Create the integrator, force reporter, and manager for the simulation.
		// Create the integrator
		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		integrator.setAccuracy(5.0e-4);
		
		// Create the force reporter
		ForceReporter* reporter = new ForceReporter(&osimModel);
		osimModel.updAnalysisSet().adoptAndAppend(reporter);
		// Create the manager
		Manager manager(osimModel, integrator);

		// Print out details of the model
		osimModel.printDetailedInfo(si, std::cout);

		// Integrate from initial time to final time
		std::clock_t startSimTime = std::clock();
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
		manager.integrate(si);
		std::cout << "Simulation time = " << 1.e3*(std::clock()-startSimTime)/CLOCKS_PER_SEC << "ms\n";

		//////////////////////////////
		// SAVE THE RESULTS TO FILE //
		//////////////////////////////

		// Save the simulation results
		// Save the states
		double outDT = 0.005;  // time interval for output reporting
		manager.getStateStorage().print("tugOfWar_afferents_states.sto", outDT);

		// Save the forces
		reporter->getForceStorage().print("tugOfWar_afferents_forces.mot", outDT);

		// Save the muscle analysis results
        IO::makeDir("MuscleAnalysisResults");
        muscAnalysis->printResults("muscle", "MuscleAnalysisResults");
		affAnalysis->printResults("afferents", "MuscleAnalysisResults", outDT);
	
		// save the controls
		osimModel.printControlStorage( "tugOfWar_controls.sto" );

		// Save the OpenSim model to a file
		osimModel.print("tugOfWar_afferents_model.osim");
	}
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    std::cout << "main() routine time = " << 1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n";

    std::cout << "OpenSim example completed successfully.\n";
	std::cin.get(); // so the console output can be read
	return 0;
}
