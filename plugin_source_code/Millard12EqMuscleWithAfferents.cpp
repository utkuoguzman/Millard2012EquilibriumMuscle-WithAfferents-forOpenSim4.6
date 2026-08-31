/* -------------------------------------------------------------------------- *
 *                 OpenSim:  Millard12EqMuscleWithAfferents.cpp               *
 * -------------------------------------------------------------------------- *
 */
 
//=============================================================================
// INCLUDES
//=============================================================================
#include "Millard12EqMuscleWithAfferents.h"
#include <iostream>  // remove later

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/*
 * Default constructor. Not used.
 */
Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents()
{
	constructProperties();
	upd_spindle().setOwnerMuscleName("no_spindle_name");
	upd_GTO().setOwnerMuscleName("no_GTO_name");
}

/*
 * Constructor.
 */
Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents(const std::string &name, 
					double maxIsometricForce, double optimalFiberLength, 
					double tendonSlackLength, double pennationAngle) : 
	Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength, 
          pennationAngle)
{
	constructProperties();
	upd_spindle().setOwnerMuscleName(getName());
	upd_GTO().setOwnerMuscleName(getName());
	
	// Initialize the work variables to calculate the acceleration
	vel = 0;
	ts[2] = 0.0; ts[1] = -0.01; ts[0] = -0.02; 
	C0 = 0.0; C1 = 0.0;
}

/*
 * Construct and initialize properties.
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
 */
void Millard12EqMuscleWithAfferents::constructProperties()
{
	setAuthors("Sergio Verduzco from code by Ajay Seth");
	constructProperty_lpf_tau(0.01); // LPF time constant
	constructProperty_spindle(Mileusnic06Spindle());
    constructProperty_GTO(Lin02GolgiTendonOrgan());
}

// Define new states and their derivatives in the underlying system
void Millard12EqMuscleWithAfferents::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	// Allow Millard2012EquilibriumMuscle to add its states, cache, etc.
	// to the system
	Super::extendAddToSystem(system);
	
	// low-pass filtered state variables used to calculate derivatives 
	addStateVariable("LPF_velocity"); // fiber velocity
	addStateVariable("LPF_acceleration"); // fiber acceleration

	// Allowing the afferent elements to become part of the system
	//spindle.extendAddToSystem(system);
	//GTO.extendAddToSystem(system);
}

void Millard12EqMuscleWithAfferents::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
	
	// I'll init the state, but not from properties
	setLPFvelocity(s, 0.0);
	setLPFacceleration(s, 0.0);


	// Initialize the work variables to calculate the acceleration
	vel = 0;
	ts[2] = 0.0; ts[1] = -0.01; ts[0] = -0.02; 
	C0 = 0.0; C1 = 0.0;
}

void Millard12EqMuscleWithAfferents::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
	

}

void Millard12EqMuscleWithAfferents::extendConnectToModel(Model& aModel)
{

	
	// The afferents need the name of their owner muscle
	upd_spindle().setOwnerMuscleName(getName());
	upd_GTO().setOwnerMuscleName(getName());
	Super::extendConnectToModel(aModel);
	// I read in Ligament.cpp that includeAsSubComponent should 
	// appear before Super::connectToModel() 
	/// I REMOVED IT. For some reason the state variables of the spindle
	/// object were not being added to the system despite this
	/// statement.
	/// includeAsSubComponent(&spindle);
	
	/// What allowed the state variables of spindle to be
	/// connected was to invoke this here.

	
	
}

//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------
void Millard12EqMuscleWithAfferents::setLPFtau(double aLPFtau) {
	set_lpf_tau(aLPFtau);
}

//--------------------------------------------------------------------------
// GET & SET States and their derivatives
//--------------------------------------------------------------------------

double Millard12EqMuscleWithAfferents::getLPFvelocity(const SimTK::State& s) const {
	return getStateVariableValue(s, "LPF_velocity");
}	
void Millard12EqMuscleWithAfferents::setLPFvelocity(SimTK::State& s, double Velocity) const {
	setStateVariableValue(s, "LPF_velocity", Velocity);
}	
double Millard12EqMuscleWithAfferents::getLPFacceleration(const SimTK::State& s) const {
	return getStateVariableValue(s, "LPF_acceleration");
}
void Millard12EqMuscleWithAfferents::setLPFacceleration(SimTK::State& s, double Acceleration) const {
	setStateVariableValue(s, "LPF_acceleration", Acceleration);
}

//=============================================================================
// COMPUTATION
//=============================================================================
void Millard12EqMuscleWithAfferents::
computeInitialFiberEquilibrium(SimTK::State& s) const
{
	// First let the muscle find an equilibrium state
	Super::computeInitialFiberEquilibrium(s);
	
	setLPFvelocity(s, getFiberVelocity(s));
	// a simplifying assumption is a steady state
	setLPFacceleration(s, 0.0); 
	
	// the spindle's initial conditions depend on the muscle,
	// so this method should be called last.
	get_spindle().computeInitialSpindleEquilibrium(s);
	
	// get a reasonable initial value for the GTO nonlinearity
	get_GTO().initFromMuscle(s);
	
	// update the work vectors assuming no acceleration
	vel[0] = vel[1] = vel[2] = getFiberVelocity(s);
	ts[2] = s.getTime();
	ts[1] = ts[2] - 0.001; ts[0] = ts[1] - 0.001;
}

void Millard12EqMuscleWithAfferents::computeStateVariableDerivatives(const SimTK::State& s) const
{
// This is the parent's computeStateVariableDerivatives
/*--------------------------------------------------------------------
	int idx = 0;

    if (!isDisabled(s)) {
        // Activation is the first state (if it is a state at all)
        if(!get_ignore_activation_dynamics() &&
           idx+1 <= getNumStateVariables()) {
               derivs[idx] = getActivationDerivative(s);
               idx++;
        }

        // Fiber length is the next state (if it is a state at all)
        if(!get_ignore_tendon_compliance() && idx+1 <= getNumStateVariables()) {
            derivs[idx] = getFiberVelocity(s);
        }
    }
--------------------------------------------------------------------*/
// This is a "carefree" version of that:
	Super::computeStateVariableDerivatives(s);
	
	// next state is the LPF velocity
	setStateVariableDerivativeValue(s, "LPF_velocity",(getFiberVelocity(s) - getLPFvelocity(s))/getLPFtau());
	 
	// the LPF acceleration
	setStateVariableDerivativeValue(s, "LPF_acceleration",(approxFiberAcceleration(s) - getLPFacceleration(s)) / getLPFtau());
	

}

//--------------------------------------------------------------------------
// Approximate the muscle fiber acceleration
//--------------------------------------------------------------------------
double Millard12EqMuscleWithAfferents::
       approxFiberAcceleration(const SimTK::State& s) const
{
	double accel;   // muscle fiber acceleration 
	double curr_vel;	//  muscle fiber velocity		
	double curr_time = s.getTime();	// time in the simulation
	
	curr_vel = getLPFvelocity(s);
	
	if( curr_time > ts(2) )
	{	// vel and ts are not ahead of current time.
		// Using 4-point Fornberg's method.
		// The formula below assumes current time = 0
		ts(0) = ts(0) - curr_time;
		ts(1) = ts(1) - curr_time;
		ts(2) = ts(2) - curr_time;
		
		// calculate coefficients
		C0(0,1) = ts(1)/(ts(1)-ts(0));
		C0(1,1) = ts(0)/(ts(0)-ts(1));
		C0(0,2) = ts(2)*C0(0,1)/(ts(2)-ts(0));
		C0(1,2) = ts(2)*C0(1,1)/(ts(2)-ts(1));
		C0(2,2) = ts(1)*ts(0)/((ts(2)-ts(0))*(ts(2)-ts(1)));
		C1(0,1) = 1/(ts(0)-ts(1));
		C1(1,1) = -C1(0,1);
		C1(2,2) = ((ts(1)-ts(0))/( (ts(2)-ts(1))*(ts(2)-ts(0)) ))
		          *(C0(1,1) - ts(1)*C1(1,1));
		C1(0,3) = C0(0,2)/ts(0);
		C1(1,3) = C0(1,2)/ts(1);
		C1(2,3) = C0(2,2)/ts(2);
		C1(3,3) = ( (ts(1)-ts(2))*(ts(2)-ts(0))/(ts(0)*ts(1)*ts(2)) )*(C0(2,2) - ts(2)*C1(2,2));
				  
		// use the coefficients
		accel = C1(3,3)*curr_vel + C1(2,3)*vel(2) + C1(1,3)*vel(1) + C1(0,3)*vel(0);
		
		// shift velocities and times
		vel(0) = vel(1); vel(1) = vel(2); vel(2) = curr_vel;
		ts(0) = ts(1) + curr_time; // changing to absolute times
		ts(1) = ts(2) + curr_time;
		ts(2) = curr_time;
	} 
	else  // computeStateVariableDerivatives was called before for a more advanced time
	{
		if( curr_time > ts(1) )
		{  // vel(1) and vel(0) still useful.
			// Using a 3-point rule for differentiation
			accel = ( 3*curr_vel - 4*vel(1) + vel(0) )/(curr_time - ts(0));
			
			// shift velocities and times
			vel(2) = curr_vel; ts(2) = curr_time;
		}
		else if( s.getTime() > ts(0) )
		{ // We only have one value before current time.
			// Using a 2-point rule for differentiation
			accel = (curr_vel - vel(0))/(curr_time - ts(0));
			
			// shift velocities and times
			vel(2) = curr_vel; vel(1) = vel(0);
			ts(2) = curr_time; ts(1) = ts(0); ts(0) = ts(1) - 1.0e-5;
		}
		else // we have no data to do this calculation
		{
			accel = getLPFacceleration(s);
			vel(2) = curr_vel; ts(2) = curr_time;
			vel(1) = vel(2); ts(1) = ts(2) - 1.0e-5;
			vel(0) = vel(1); ts(0) = ts(1) - 1.0e-5;
			//std::cout << "computed acceleration with no data \n";
		}
	}
	return accel;
}
