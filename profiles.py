from unittest import case
import numpy as np


class FootForceProfile:
    """Class to generate foot force profiles over time using a single CPG oscillator"""

    def __init__(self, mode: str):
        """
        Create instance of foot force profile with its arguments.

        Args:
            mode (str): Mode of the force profile ('foward', 'lateral', 'spin')
            f0 (float): Frequency of the impulse (Hz)
            f1 (float): Frequency between impulses (Hz)             <- PAS SUR DE L'UTILITE
            Fx (float): Foot force amplitude in X direction (N)
            Fy (float): Foot force amplitude in Y direction (N)
            Fz (float): Foot force amplitude in Z direction (N)
        """
        self.theta = 0
        self.mode = mode
        #TUNING PARAMETERS
        match mode:
            case 'none': 
                f0=0 
                f1=0
                Fx=0
                Fy=0
                Fz=0
                phase_offset = 0.0
                phase_offset_side = 0.0

            case 'forward': # CHANGE in quadruped_jump.py, nominal_position :des_pos = [0,0,-0.2] et hip_offset = 0.1
                ### Params without optimization
                f0= 1.5
                f1 = 1
                Fx = 60
                Fy = 0
                Fz = 150

                phase_offset = 0.0
                phase_offset_side = 0.0
                
                # ### Params with optimization
                # f0=1.56
                # f1= 1.64
                # Fx= 99.98
                # Fy=0
                # Fz=199.10

                # phase_offset = 0.0
                # phase_offset_side = 0.0

            case 'lateral': # CHANGE in quadruped_jump.py, nominal_position :des_pos = [0,0,-0.15] et hip_offset = 0.1

                # ### Params without optimization
                # f0= 3
                # f1= 0.3 # changer les bornes
                # Fx= -10
                # Fy= 40
                # Fz= 150

                # phase_offset = 0.0
                # phase_offset_side = 0.0

                ### Params with optimization
                f0=2.913407985542411
                f1=0.5043077082354251
                Fx=2.700764507784139
                Fy=82.5129054728558
                Fz=125.8335580297119

                phase_offset = 0.0
                phase_offset_side = 0.0

            case 'spin': # CHANGE in quadruped_jump.py, nominal_position :des_pos = [0,0,-0.2] et hip_offset = 0.1
                         # + in apply_force_profile change rotation_profil = 'clockwise' or 'anticlockwise'
                f0=1.51
                f1=3.01
                Fx=0
                Fy=87
                Fz=124.30

                phase_offset = 0.0
                phase_offset_side = 0.0
        
        self.f0 = f0
        self.f1 = f1
        self.F = np.array([Fx, Fy, Fz])
        self.phase_offset = phase_offset
        self.phase_offset_sides = phase_offset_side
        self.mode = mode

    def step(self, dt: float):
        """
        Step the oscillator by a single timestep.

        Args:
            dt (float): Timestep duration (s)
        """
        # TODO: integrate the oscillator equation
        #Calculer le temps d'un step (dt):
        self.phase()  # mettre a jour theta avant de l'utiliser
        if 0 <= self.theta < np.pi: #impulse phase
            self.theta += 2 * np.pi * self.f1 * dt # dtheta/dt = theta' = 2pi*f1 
        else:
            self.theta += 2 * np.pi * self.f0 * dt #dtheta/dt = theta' = 2pi*f0
        
        return 0
    




    def phase(self) -> float:
        """Get oscillator phase in [0, 2pi] range."""
        # TODO: return the phase of the oscillator in [0, 2pi] range
        self.theta = self.theta % (2 * np.pi) # theta modulo 2pi -> phase dans [0, 2pi]
        #
        return 0

    def force(self) -> np.ndarray:
        """
        Get force vector of the force profile at the current timestep.

        Returns:
            np.ndarray: An R^3 array [Fx, Fy, Fz]
        """
        # TODO: return the force vector given the oscillator state
        force = np.zeros(3)
        match self.mode:
            case 'none':
                force[0] = 0
                force[1] = 0
                force[2] = 0

            case 'forward':
                #OSCILLATEUR SIMPLE EN X
                force[0] = self.F[0] * np.sin(self.theta + np.pi/4)
                force[0] = np.clip(force[0], None, 0)  # Upper bound to 0 -> no pulling on the ground
                #OSCILLATEUR SIMPLE EN Y
                force[1] = 0
                #OSCILLATEUR SIMPLE EN Z
                force[2] = self.F[2] * np.sin(self.theta)
                force[2] = np.clip(force[2], None, 0)

            case 'lateral':
                #OSCILLATEUR SIMPLE EN X
                force[0] = self.F[0] * np.sin(self.theta)  
                force[0] = np.clip(force[0], 0, None)
                #OSCILLATEUR SIMPLE EN Y
                force[1] = self.F[1] * np.sin(self.theta)
                force[1] = np.clip(force[1], None, 0) 
                #OSCILLATEUR SIMPLE EN Z
                force[2] = self.F[2] * np.sin(self.theta )
                force[2] = np.clip(force[2], None, 0) 
           
            case 'spin':
                #OSCILLATEUR SIMPLE EN X
                force[0] = 0
                #OSCILLATEUR SIMPLE EN Y
                force[1] = self.F[1] * np.sin(self.theta + np.pi/4)
                force[1] = np.clip(force[1], None, 0)
                #OSCILLATEUR SIMPLE EN Z
                force[2] = self.F[2] * np.sin(self.theta )
                force[2] = np.clip(force[2], None, 0)

        return force

    def impulse_duration(self) -> float: #impulse = phase where force is applied
        """Return impulse duration in seconds."""
        # TODO: compute the impulse duration in seconds
        impulse_duration = 1 / self.f0 if self.f0 != 0 else 0
        return impulse_duration
    
    def idle_duration(self) -> float: #idle = phase where no force is applied
        """Return idle time between impulses in seconds"""
        # TODO: compute the idle duration in seconds
        idle_duration = 1/ self.f1 if self.f1 != 0 else 0
        return idle_duration
