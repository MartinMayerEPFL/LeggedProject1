import numpy as np
from env.simulation import QuadSimulator, SimulationOptions
import matplotlib.pyplot as plt
from profiles import FootForceProfile
import time

N_LEGS = 4
N_JOINTS = 3


def quadruped_jump():
    # Initialize simulation
    # Feel free to change these options! (except for control_mode and timestep)
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=True,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)

    # Determine number of jumps to simulate
    n_jumps = 7  # Feel free to change this number
    jump_duration = 2.0  # TODO: determine how long a jump takes
        ###Comment  determiner la durée d'un saut ?
    # Compute number of simulation steps
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)
    # TODO: set parameters for the foot force profile here
    

    force_profile = FootForceProfile('lateral') # none, forward, lateral, spin
        ### Comment choisir ?
    pied0_array = []
    pied1_array = []
    pied2_array = []
    pied3_array = []
    array_vx = []
    array_vy = []
    array_vz = []
    array_posx = []
    array_posy = []
    array_posz = []
    profil_force_x =[]
    profil_force_y = []
    profil_force_z = []
    array_theta = []
    hip_speed_avant = []
    hip_speed_arriere = []

    for _ in range(n_steps):
        
        # If the simulator is closed, stop the loop
        if not simulator.is_connected():
            break
        
        array_vx.append(simulator.get_base_linear_velocity()[0])
        array_vy.append(simulator.get_base_linear_velocity()[1])
        array_vz.append(simulator.get_base_linear_velocity()[2])
        array_posx.append(simulator.get_base_position()[0])
        array_posy.append(simulator.get_base_position()[1])
        array_posz.append(simulator.get_base_position()[2])
        profil_force_x.append(force_profile.force()[0])
        profil_force_y.append(force_profile.force()[1])
        profil_force_z.append(force_profile.force()[2])       
        pied0_array.append(simulator.get_foot_contacts()[0])
        pied1_array.append(simulator.get_foot_contacts()[1])
        pied2_array.append(simulator.get_foot_contacts()[2])
        pied3_array.append(simulator.get_foot_contacts()[3])
        array_theta.append(force_profile.theta)
        hip_speed_avant.append(simulator.get_motor_velocities(0)[0])
        hip_speed_arriere.append(simulator.get_motor_velocities(2)[0])
         ### Comment choisir ?
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

    fig, axs = plt.subplots(nrows=4, ncols=1, figsize=(10, 8))
    axs[0,].plot(pied0_array, label='Pied 0', color='red')
    axs[0,].set_title('Foot 0 Contact')
    axs[1,].plot(pied1_array, label='Pied 1', color='green')
    axs[1,].set_title('Foot 1 Contact')
    axs[2,].plot(pied2_array, label='Pied 2', color='blue')
    axs[2,].set_title('Foot 2 Contact')
    axs[3,].plot(pied3_array, label='Pied 3', color='orange')
    axs[3,].set_title('Foot 3 Contact')
    fig.tight_layout()
    plt.show()

    plt.plot(array_posx, label='x')
    plt.plot(array_posy, label='y')
    plt.plot(array_posz, label='z')
    plt.xlabel("Time step")
    plt.ylabel("Base position (m)")
    plt.title("Position")
    plt.grid()
    plt.legend()
    plt.show()

    plt.plot(array_vx, label='Vx')
    plt.plot(array_vy, label='Vy')
    plt.plot(array_vz, label='Vz')
    plt.xlabel("Time step")
    plt.ylabel("Instant velocity (m/s)")
    plt.title("Velocity")
    plt.grid()
    plt.legend()
    plt.show()

    plt.plot(hip_speed_avant, label='Hip Speed Front')
    plt.plot(hip_speed_arriere, label='Hip Speed rear')
    plt.xlabel("Time step")
    plt.ylabel("Motor Speed (rad/s)")
    plt.title("Motor Speeds, fr/rr")
    plt.grid()
    plt.legend()
    plt.show()

    plt.plot(array_theta, label='Theta')
    plt.xlabel("Time step")
    plt.ylabel("Theta (rad)")
    plt.title("Theta")
    plt.grid()
    plt.legend()
    plt.show()
    

def nominal_position(
    simulator: QuadSimulator, 
    KpCartesian=np.diag([800,900,1000]), 
    KdCartesian=np.diag([35,35,40]), 
    des_pos=np.array([0,0,-0.15]),
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
            des_pos[1] = -hip_offset
        elif leg_id == 1 or leg_id == 3 :  # left legs
            des_pos[1] = hip_offset
            
        #    des_pos[1] = des_pos[1] + hip_offset

        tau_i = J_T @ (KpCartesian @ (des_pos - foot_pos) + KdCartesian @ (des_vel - foot_vel))

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
        K_vmc = 2

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

        # décalage de phase entre les pattes avant et arrière
        phase_offset =0 # -np.pi/5.7  # mettre 0 si forward

        if leg_id == 0:  # front legs
            force_profile.theta += phase_offset
        elif leg_id == 2:  # back legs
            force_profile.theta -= phase_offset

        phase_offset_sides = 0
        if leg_id == 1 or leg_id == 3:  # left legs
            force_profile.theta += phase_offset_sides
        elif leg_id == 0 or leg_id == 2:  # right legs
            force_profile.theta -= phase_offset_sides

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

        #print ("Force applied on leg ", leg_id, " : ", Ftwist)

        tau_i = J_T @Ftwist

        #else:
        #    tau_i = J_T @ +force_profile.force()
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


if __name__ == "__main__":
    quadruped_jump()
