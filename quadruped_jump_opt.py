import optuna
import numpy as np
from functools import partial
from optuna.trial import Trial
from env.simulation import QuadSimulator, SimulationOptions
import matplotlib.pyplot as plt

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
        tracking_camera=False,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)

    # Create a maximization problem
    objective = partial(evaluate_jumping, simulator=simulator)
    sampler = optuna.samplers.TPESampler(seed=29) #SEED VALUE
    study = optuna.create_study(
        study_name="Quadruped Jumping Optimization",
        sampler=sampler,
        direction="maximize",
    )

    # Run the optimization
    # You can change the number of trials here
    study.optimize(objective, n_trials=250)

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


    f0_opt = trial.suggest_float(name="f0 : ", low=2.8, high=3.5)
    f1_opt = trial.suggest_float(name="f1 : ", low=0.4, high=0.6)
    Fx_opt=trial.suggest_float(name="Fx : ", low=-20.0, high=20.0)
    Fy_opt=trial.suggest_float(name="Fy : ", low=30, high=90.0)
    Fz_opt=trial.suggest_float(name="Fz : ", low=120, high=200.0)

    # Reset the simulation
    simulator.reset()

    # Extract simulation options
    sim_options = simulator.options

    # Determine number of jumps to simulate
    n_jumps = 7  # Feel free to change this number
    jump_duration = 2  # TODO: determine how long a jump takes
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)

    # TODO: set parameters for the foot force profile here
    force_profile = FootForceProfile('lateral')
    force_profile.f0 = f0_opt
    force_profile.f1 = f1_opt
    force_profile.F[0] = Fx_opt
    force_profile.F[1] = Fy_opt
    force_profile.F[2] = Fz_opt


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


    distance_x, distance_y, distance_z = simulator.get_base_position()
    speed_x, _, _ = simulator.get_base_linear_velocity()
    roll,_,yaw = simulator.get_base_orientation_roll_pitch_yaw()


    return abs(distance_y)*5 -abs(distance_x)*3 - abs(np.sin(yaw))*20 - np.exp((abs(roll)-np.pi/2)*10)


if __name__ == "__main__":
    quadruped_jump_optimization()
