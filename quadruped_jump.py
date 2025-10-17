import numpy as np
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile

N_LEGS = 4
N_JOINTS = 3


def quadruped_jump():
    # Initialize simulation
    # Feel free to change these options! (except for control_mode and timestep)
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=True,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=False,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)

    # Determine number of jumps to simulate
    n_jumps = 10  # Feel free to change this number
    jump_duration = 5.0  # TODO: determine how long a jump takes
        ###Comment  determiner la durée d'un saut ?
    # Compute number of simulation steps
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)
    # TODO: set parameters for the foot force profile here
    

    force_profile = FootForceProfile('lateral') # none, forward, lateral, spin
        ### Comment choisir ?


    for _ in range(n_steps):
        # If the simulator is closed, stop the loop
        if not simulator.is_connected():
            break

        # Step the oscillator
        force_profile.step(sim_options.timestep)

        # Compute torques as motor targets
        # The convention is as follows:
        # - A 1D array where the torques for the 3 motors follow each other for each leg
        # - The first 3 elements are the hip, thigh, calf torques for the FR leg.
        # - The order of the legs is FR, FL, RR, RL (front/rear,right/left)
        # - The resulting torque array is therefore structured as follows:
        # [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf]
        tau = np.zeros(N_JOINTS * N_LEGS)

        # TODO: implement the functions below, and add potential controller parameters as function parameters here
        tau += nominal_position(simulator)
        tau += apply_force_profile(simulator, force_profile)
        tau += gravity_compensation(simulator)

        # If touching the ground, add virtual model
        foot_contact = 0
        foot_contact = simulator.get_foot_contacts().sum()
        if foot_contact >= 2:
            tau += virtual_model(simulator)


        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()

    # Close the simulation
    simulator.close()

    # OPTIONAL: add additional functions here (e.g., plotting)
    

def nominal_position(
    simulator: QuadSimulator, 
    Kpjoin=np.diag([400,400,400]), 
    Kdjoin=np.diag([35,35,35]), 
    des_pos=np.array([0,0,-0.25]),
    des_vel=np.array([0,0,0])
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute nominal position torques for leg_id
        tau_i = np.zeros(3)
        J, foot_pos = simulator.get_jacobian_and_position(leg_id)
        J_T = J.transpose()

        foot_vel = J @ simulator.get_motor_velocities(leg_id)

        # hip offset
        hip_offset = 0.1
        
        if leg_id == 0 or leg_id == 2 :  # right legs
            des_pos[1] = - hip_offset
        elif leg_id == 1 or leg_id == 3 :  # left legs
            des_pos[1] = hip_offset
            
        #    des_pos[1] = des_pos[1] + hip_offset

        tau_i = J_T @ (Kpjoin @ (des_pos - foot_pos) + Kdjoin @ (des_vel - foot_vel))

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    return tau


def virtual_model(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute virtual model torques for leg_id

        tau_i = np.zeros(3)

        first_matrix = [
            [1, 1, -1, -1],
            [-1, 1, -1, 1],
            [0, 0, 0, 0]
        ]
        P = simulator.get_base_orientation_matrix() @ first_matrix
        
        # Gain que l'on peut changer 
        K_vmc = 8

        matrix_K = K_vmc*([0, 0, 1] @ P)
        zeros_part = np.zeros((2, 4))
        F_vmc = np.vstack([zeros_part, matrix_K])
        #
        #           | 0  0  0  0 |
        # F_vmc =   | 0  0  0  0 | 
        #           | ka kb kc kd|
        #

        # tau_i = J_i(q_i) * F_vmc_i
        jacobian_inverse = np.linalg.inv(simulator.get_jacobian_and_position(leg_id)[0]) # Jacobienne inverse
        tau_i += jacobian_inverse @ F_vmc[: , leg_id]
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    return tau


def gravity_compensation(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute gravity compensation torques for leg_id
        tau_i = np.zeros(3)
        J, foot_pos = simulator.get_jacobian_and_position(leg_id)
        J_T = J.transpose()

        g = np.zeros(3)
        g = np.array([0,0, -9.81*simulator.get_mass()/4]) #gravity force on each leg


        tau_i += J_T @ g
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


def apply_force_profile(
    simulator: QuadSimulator,
    force_profile: FootForceProfile,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute force profile torques for leg_id
        tau_i = np.zeros(3)

        J, _ = simulator.get_jacobian_and_position(leg_id)
        J_T = J.transpose()

        #IF SPIN -> CHOOSE ROTATION PROFIL
        rotation_profil = 'none' #none, clockwise, anticlockwise

        Ftwist = force_profile.force()
        if rotation_profil == 'clockwise':
            if leg_id == 0 or leg_id == 1:
                Ftwist[1] = Ftwist[1]
            else :
                Ftwist[1] = -Ftwist[1]
        elif rotation_profil == 'anticlockwise':
            if leg_id == 0 or leg_id == 1:
                Ftwist[1] = -Ftwist[1]
            else :
                Ftwist[1] = Ftwist[1]

        tau_i = J_T @Ftwist

        #else:
        #    tau_i = J_T @ +force_profile.force()
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


if __name__ == "__main__":
    quadruped_jump()
