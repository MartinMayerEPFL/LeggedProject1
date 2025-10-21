import optuna
import numpy as np
from functools import partial
from optuna.trial import Trial
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile

from quadruped_jump import (
    nominal_position,
    gravity_compensation,
    apply_force_profile,
    virtual_model,
)


N_LEGS = 4
N_JOINTS = 3


def quadruped_jump_optimization():
    # Initialize simulation
    # Feel free to change these options! (except for control_mode and timestep)
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=False,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)

    # Create a maximization problem
    objective = partial(evaluate_jumping, simulator=simulator)
    sampler = optuna.samplers.TPESampler(seed=8) #SEED VALUE
    study = optuna.create_study(
        study_name="Quadruped Jumping Optimization",
        sampler=sampler,
        direction="maximize",
    )

    # Run the optimization
    # You can change the number of trials here
    study.optimize(objective, n_trials=20)

    # Close the simulation
    simulator.close()

    # Log the results
    print("Best value:", study.best_value)
    print("Best params:", study.best_params)

    # OPTIONAL: add additional functions here (e.g., plotting, recording to file)
    # E.g., cycling through all the evaluated parameters and values:
    for trial in study.get_trials():
        trial.number  # Number of the trial
        trial.params  # Used parameters
        trial.value  # Resulting objective function value


def evaluate_jumping(trial: Trial, simulator: QuadSimulator) -> float:

    # TODO: pick optimization variables
    # The following function creates an optimization variable with given name and lower and upper bounds
    # You can then plug in the value in your controller
    
    #KpCartesian = np.diag([800,900,1000])
    #KdCartesian = np.diag([35,35,40])
    #K_vmc = 2

    # f0=1.9254848285487531
    # f1=4.600677008247974
    # Fx=25.391060416989895
    # Fy=0
    # Fz=186.27433930339313
    # self.phase_offset = -0.4034946806277691

    # Best value: 1.392441231119883
    # Best params: {'f0 : ': 1.7158454906731597, 'f1 : ': 7.516423479880351, 'Fx : ': -8.30410949376508, 'Fy : ': 45.95849191581006, 'Fz : ': 171.2253840683992, 'phase_offset : ': 0.0, 'phase_offset_side : ': -0.0}

    f0_opt = trial.suggest_float(name="f0 : ", low=1.6, high=2.0)
    f1_opt = trial.suggest_float(name="f1 : ", low=1.0, high=2.0)
    Fx_opt=trial.suggest_float(name="Fx : ", low=0.0, high=0.0)
    Fy_opt=trial.suggest_float(name="Fy : ", low=15.0, high=40.0)
    Fz_opt=trial.suggest_float(name="Fz : ", low=120.0, high=160.0)
    
    phase_offset_opt = trial.suggest_float(name="phase_offset : ", low=0.0, high=0.0)
    phase_offset_side_opt = trial.suggest_float(name="phase_offset_side : ", low=0.0, high=0.0)

    # Reset the simulation
    simulator.reset()

    # Extract simulation options
    sim_options = simulator.options

    # Determine number of jumps to simulate
    n_jumps = 1  # Feel free to change this number
    jump_duration = 10.0  # TODO: determine how long a jump takes
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)

    # TODO: set parameters for the foot force profile here
    force_profile = FootForceProfile('lateral')
    force_profile.f0 = f0_opt
    force_profile.f1 = f1_opt
    force_profile.F[0] = Fx_opt
    force_profile.F[1] = Fy_opt
    force_profile.F[2] = Fz_opt

    force_profile.phase_offset = phase_offset_opt
    force_profile.phase_offset_sides = phase_offset_side_opt

#[I 2025-10-20 16:53:58,391] Trial 58 finished with value: -29.072471427972893 and parameters: {'f0 : ': 2.577853284635687, 'f1 : ': 15.933827826378465, 'Fx : ': -8.478171543032063, 'Fy : ': 39.09683001551307, 'Fz : ': 100.86074959624929, 'phase_offset : ': -1.2493778336957861, 'phase_offset_side : ': -1.415806927004636}. Best is trial 32 with value: -8.404384893149427.
    

    for _ in range(n_steps):
        # Step the oscillator
        force_profile.step(sim_options.timestep)

        # Compute torques as motor targets (reuses your controller functions)
        # OPTIONAL: add potential extra controller parameters here
        tau = np.zeros(N_JOINTS * N_LEGS)
        tau += nominal_position(simulator)
        tau += apply_force_profile(simulator, force_profile)
        tau += gravity_compensation(simulator)

        # If touching the ground, add virtual model
        # TODO: how do we know we're on the ground?
        foot_contact = 0
        foot_contact = simulator.get_foot_contacts().sum()
        if foot_contact >= 2:
            tau += virtual_model(simulator)

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()

    # TODO: implement an objective function and return its value
    # Note: the objective function is maximized!

    ### OPTIMAZER FOR FORWARD JUMP
    # distance_x, _, _ = simulator.get_base_position()
    # roll , pitch, _ = simulator.get_base_orientation_roll_pitch_yaw()

    # return distance_x - np.exp((abs(pitch)-np.pi/2)*10) - np.exp((abs(roll)-np.pi/2)*10)

    # ### OPTIMAZER FOR LATERAL JUMP
    distance_x, distance_y, _ = simulator.get_base_position()

    roll, _, yaw = simulator.get_base_orientation_roll_pitch_yaw()

    return abs(distance_y)*5 -abs(distance_x)*3 - abs(np.sin(yaw))*10 - np.exp((abs(roll)-np.pi/2)*10)


if __name__ == "__main__":
    quadruped_jump_optimization()
