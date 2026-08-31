import org.opensim.modeling as modeling

# Define where to save the log file
log_file = 'D:/simulation_log.txt'

def log(message):
    # 'a' mode appends to the file immediately and closes it safely
    f = open(log_file, 'a')
    f.write(message + '\n')
    f.close()

# Clear the log file for a fresh run
f = open(log_file, 'w')
f.write("--- STARTING NEW SIMULATION RUN ---\n")
f.close()

log("1. Getting model...")
model = getCurrentModel()

log("2. Initializing system (setting up states)...")
state = model.initSystem()

log("3. Configuring CPodes Manager...")
manager = modeling.Manager(model)
manager.setIntegratorMethod(7)
manager.setIntegratorAccuracy(1.0e-2)
manager.setIntegratorMaximumStepSize(0.01)   
manager.setIntegratorMinimumStepSize(1.0e-7) 
manager.setIntegratorInternalStepLimit(500000)

log("4. Manager initialize() - WARNING: CPodes is building the Jacobian here, this might take time!")
manager.initialize(state)

target_time = 1.0 
chunk_size = 0.01  # Change this from 0.1 to 0.01

current_time = manager.getState().getTime()
log("5. SUCCESS! Simulation starting. Target time: " + str(target_time) + "s")

while current_time < target_time:
    next_step = min(current_time + chunk_size, target_time)
    
    manager.integrate(next_step)
    
    current_time = manager.getState().getTime()
    log("--> Progress: " + str(current_time) + " / " + str(target_time) + " seconds complete")

log("6. Simulation finished completely!")

statesTable = manager.getStatesTable()
save_path = 'D:/Documents/OpenSim Files/Models/SoccerKick/SoccerKick_CPodes.sto'
modeling.STOFileAdapter.write(statesTable, save_path)
log("7. Results saved successfully.")