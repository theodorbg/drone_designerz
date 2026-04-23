import matplotlib.pyplot as plt
import numpy as np
from numpy import exp, sqrt, pi

# GLOBAL PARAMETERS

class Planet:
    def __init__(self, name, g, altitude=1, rho=None):
        self.name = name
        self.g = g
        self.altitude = altitude
        if rho is None:
            self.rho = self.compute_rho(altitude)
        else:
            self.rho = rho
    
    def compute_rho(self, altitude):
        
        T = -31 - 0.000998 * altitude
        p = 0.699 * exp(-0.00009*altitude)
        
        rho = p/(0.1921 * (T + 273.1))
        return rho


class Drone:
    """
    Drone class to encapsulate all properties and methods related to the drone design and performance evaluation.
    
    notes:
    C_D0 is assumed 0.02 everywhere (from pdf assignment)
    gamma is assumed 1.15 for now (from lecture)
    Power formulas must be revisited for a more detailed calculation
        
        
    """
    def __init__(self, name: str, mass: float, fuselage_mass: float, payload_mass: float, 
                 battery_mass: float, aux_components_mass: float, motor_mass: float, rotor_mass: float,
                 N_batteries: int, total_battery_capacity: float,
                 rotor_diameter: float, chord: float, N_blades: int, N_rotors: int,
                 rpm: int,
                 peak_power: float, avg_power: float,
                 C_D0: float, gamma: float):
        
        
        
        # MASS PROPERTIES
        self.name = name
        self.mass = mass # total mass of drone
        self.fuselage_mass = fuselage_mass
        self.mass_no_fuselage = self.mass - self.fuselage_mass
        self.payload_mass = payload_mass
        self.battery_mass = battery_mass
        self.aux_components_mass = aux_components_mass
        self.motor_mass = motor_mass
        self.rotor_mass = rotor_mass # for ingenuity this is total 70g with two rotors
        self.blade_mass = rotor_mass / (N_blades * N_rotors) # mass of one blade
        
        # update mass based on payload 
        self.mass += self.payload_mass
        
        # MASS PROPERTIES TO BE COMPUTED
        # self.rotor_mass = 0
        # self.total_rotor_mass = self.rotor_mass * N_rotors
        
        # BATTERY PROPERTIES
        self.N_batteries = N_batteries
        self.total_battery_capacity_Wh = total_battery_capacity # Wh
        self.total_battery_capacity = self.total_battery_capacity_Wh * 3600 # J
        self.battery_capacity_per_battery = self.total_battery_capacity / N_batteries # J per battery
        
        # ROTOR PROPERTIES
        self.rotor_diameter = rotor_diameter
        self.rotor_radius = rotor_diameter / 2
        self.rotor_area = pi * self.rotor_radius**2
        self.chord = chord
        self.blade_area = chord * self.rotor_radius
        self.N_blades = N_blades
        self.N_rotors = N_rotors
        
        
        # ROTATIONAL PROPERTIES
        self.rpm = rpm
        self.omega = 2 * pi * rpm / 60
        
        # POWER PROPERTIES
        self.peak_power = peak_power
        self.avg_power = avg_power
        
        # AERODYNAMIC PROPERTIES
        self.C_D0 = C_D0 # zero-lift drag coefficient 
        self.gamma = gamma # induced-loss correction factor (assumed 1.15 in this course) )
        
        
        # ATTRIBUTES TO BE COMPUTED
        self.total_thrust = None
        self.C_T = None
        self.ideal_power = None
        self.power_loss = None
        self.solidity = None
        self.Cp = None
        self.hover_power = None
        self.total_hover_time = None
        
        
    def compute_thrust(self, g):
        self.total_thrust = self.mass * g
    
    def compute_C_T(self, rho):
        T = self.total_thrust
        A = self.rotor_area
        omega = self.omega
        R = self.rotor_radius
        
        self.C_T = T / (0.5 * rho * A * (omega * R)**2)
        
    
    def compute_ideal_power(self, rho):
        
        # equation 16
        
        T = self.total_thrust
        A = self.rotor_area
                
        if T <= 0 or rho <= 0 or A <= 0:
            self.ideal_power = np.nan
        else:
            # numerically safer than T**(3/2)
            self.ideal_power = (T * np.sqrt(T)) / np.sqrt(2.0 * rho * A)
        
    
    def compute_power_loss(self, rho):
        
        #TODO LOOK INTO THIS FORMULA, MAYBE WE NEED FACTOR OF 2 IN FRON OF THIS FOR TWO ROTORS?
        # equation from slide 7 lecture 10a blade design
        
        c = self.chord
        Nb = self.N_blades
        Cd0 = self.C_D0
        omega = self.omega
        R = self.rotor_radius
        self.power_loss = 1/8 * rho * c * Nb * Cd0 * omega**3 * R**4 # * N ROTORS???
        
    
    def compute_local_solidity(self):
        Nb = self.N_blades
        c = self.chord
        R = self.rotor_radius
        
        self.solidity = Nb * c / (pi * R)
        
    
    def compute_Cp(self):
        
        gamma = self.gamma
        C_T = self.C_T
        sigma = self.solidity
        CD0 = self.C_D0
        
        if C_T < 0 or not np.isfinite(C_T):
            self.Cp = np.nan
        else:
            # numerically safer than C_T**(3/2)
            self.Cp = 0.5 * gamma * (C_T * np.sqrt(C_T)) + 0.25 * sigma * CD0
       
    def compute_hover_power(self):
        gamma = self.gamma
        P_ideal = self.ideal_power
        P0 = self.power_loss
        
        self.hover_power = gamma * P_ideal + P0
    
    def planet_performance(self, planet):
        g = planet.g
        rho = planet.rho
        
        self.compute_thrust(g)
        self.compute_C_T(rho)
        self.compute_ideal_power(rho)
        self.compute_power_loss(rho)
        self.compute_local_solidity()
        self.compute_Cp()
        self.compute_hover_power()
        self.compute_flight_time()
    
    def compute_flight_time(self):
        # flight time in hours from battery capacity and hover power
        self.total_hover_time = (self.N_batteries * self.battery_capacity_per_battery) / self.hover_power

    def solve_mass_power(self, P_initial: float, ingenuity: 'Drone', planet: 'Planet', tol: float=1e-4, max_iter: int=1000, alpha: float=0.5):
        
        
            P_drone = P_initial
            
            
            for i in range(max_iter):
                
                # 1. Estimate mass components that depend on P_drone
                self.motor_mass = self.N_rotors * (ingenuity.motor_mass/ ingenuity.N_rotors) * (P_drone / ingenuity.hover_power) # kg

                # 2. Compute the mass of the drone without fuselage
                self.mass_no_fuselage = self.payload_mass + self.battery_mass + self.aux_components_mass + self.rotor_mass + self.motor_mass
                
                # 3. compute the mass of the fuselage, by the linear relationship with ingenuity
                self.fuselage_mass = self.mass_no_fuselage * (ingenuity.fuselage_mass / ingenuity.mass_no_fuselage)
                
                # 4. Compute the total mass of the drone
                self.mass = self.mass_no_fuselage + self.fuselage_mass            
                
                # 5. Compute the hover power for this design
                self.planet_performance(planet)
                P_new = self.hover_power
                
                # 6. Check convergence
                if abs(P_new - P_drone) < tol:
                    # print(f"Converged in {i+1} iterations")
                    return self
                
                # 7. Relaxed update — blend old and new estimate to avoid oscillation
                P_drone = alpha * P_new + (1 - alpha) * P_drone
            
            print(f"Warning: did not converge after {max_iter} iterations. Residual = {abs(P_new - P_drone):.4f} W")
            return None

        

# TODO LOOK INTO POWER LOSS FORMULA
# TODO I THINK THE TOTAL POWER IS FOR ONE ROTOR, SO WE NEED A FACTOR OF 2 THERE AS WELL (SEE OLD Q1.PY CODE
# TODO WE ALSO NEED TO IMPLEMENT A VERSION WITH MORE REALISTIC ASSUMPTIONS)



