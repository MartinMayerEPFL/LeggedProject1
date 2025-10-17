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
            case 'forward':
                f0=1.25 
                f1=20
                Fx=60
                Fy=0
                Fz=140
            case 'lateral':
                f0=1
                f1=7
                Fx=0
                Fy=30
                Fz=100
            case 'spin':
                f0=1.25
                f1=20
                Fx=40
                Fy=40
                Fz=140
        ###
        self.f0 = f0
        self.f1 = f1
        self.F = np.array([Fx, Fy, Fz])
        self.mode = mode

    def step(self, dt: float):
        """
        Step the oscillator by a single timestep.

        Args:
            dt (float): Timestep duration (s)
        """
        # TODO: integrate the oscillator equation
        #Calculer le temps d'un step (dt):

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
            case 'forward':
                force[0] = self.F[0] * np.sin(self.theta+ np.pi/4)  #OSCILLATEUR SIMPLE EN X
                force[0] = np.clip(force[0], None, 0)  # Upper bound to 0 -> no pulling on the ground
                #OSCILLATEUR SIMPLE EN Y -> Pas forcement utile bizarre d'avoir un profil en Y 
                #force[0] = self.F[1] * np.sin(self.theta)
                force[1] = 0
                #OSCILLATEUR SIMPLE EN Z
                force[2] = self.F[2] * np.sin(self.theta)
                force[2] = np.clip(force[2], None, 0)  # Upper bound to 0 -> no pulling on the ground
            case 'lateral':
                force[0] = 0
                #OSCILLATEUR SIMPLE EN Y
                force[1] = self.F[1] * np.sin(self.theta + np.pi/6)
                #force[1] = np.clip(force[1], None, 0)  # Upper bound to 0 -> no pulling on the ground
                #OSCILLATEUR SIMPLE EN Z
                force[2] = self.F[2] * np.sin(self.theta)
                force[2] = np.clip(force[2], None, 0)  # Upper bound to 0 -> no pulling on the ground
           
            case 'spin':
                force[0] = self.F[0] * np.sin(self.theta + np.pi/4)
                force[0] = np.clip(force[0], None, 0)  # Upper bound to 0 -> no pulling on the ground
                force[1] = self.F[1] * np.sin(self.theta + np.pi/4)
                force[1] = np.clip(force[1], None, 0)  # Upper bound to 0 -> no pulling on the ground
                force[2] = self.F[2] * np.sin(self.theta)
                force[2] = np.clip(force[2], None, 0)  # Upper bound to 0 -> no pulling on the ground
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
