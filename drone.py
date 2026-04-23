import matplotlib.pyplot as plt
import numpy as np
from numpy import exp, sqrt, pi
from planet import Planet


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
        
        # ROTOR PROPERTIES
        self.rotor_diameter = rotor_diameter
        self.rotor_radius = rotor_diameter / 2
        self.rotor_area = pi * self.rotor_radius**2
        self.chord = chord
        self.blade_area = chord * self.rotor_radius
        self.N_blades = N_blades
        self.N_rotors = N_rotors
        
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
        self.blade_mass = rotor_mass / (self.N_blades * self.N_rotors) # mass of one blade
        
        # update mass based on payload 
        self.mass += self.payload_mass
        
        # BATTERY PROPERTIES
        self.N_batteries = N_batteries
        self.total_battery_capacity_Wh = total_battery_capacity # Wh
        self.total_battery_capacity = self.total_battery_capacity_Wh * 3600 # J
        self.battery_capacity_per_battery = self.total_battery_capacity / N_batteries # J per battery
        
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
        T_per_rotor = T / self.N_rotors
        A = self.rotor_area # Area of ONE rotor
        N_rotors = self.N_rotors
        omega = self.omega
        R = self.rotor_radius
        
        # C_T per rotor: thrust per rotor / dynamic pressure
        self.C_T = T_per_rotor / (0.5 * rho * A * (omega * R)**2)
 
    def compute_ideal_power(self, rho):
        T = self.total_thrust
        A = self.rotor_area           # area of ONE rotor
        N = self.N_rotors

        if T <= 0 or rho <= 0 or A <= 0:
            self.ideal_power = np.nan
            print(f"Warning: Invalid parameters for ideal power calculation. T={T}, rho={rho}, A={A}")
        else:
            T_per_rotor = T / N
            # ideal power for ALL rotors combined
            self.ideal_power = N * (T_per_rotor * np.sqrt(T_per_rotor)) / np.sqrt(2.0 * rho * A)

    def compute_power_loss(self, rho):
        # Profile power loss per rotor, then sum over rotors
        c = self.chord
        Nb = self.N_blades
        Cd0 = self.C_D0
        omega = self.omega
        R = self.rotor_radius

        p0_per_rotor = 1/8 * rho * c * Nb * Cd0 * omega**3 * R**4
        self.power_loss = self.N_rotors * p0_per_rotor
           
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

    def solve_mass_power(self, P_initial: float, ingenuity: 'Drone', planet: 'Planet',
                         tol: float=1e-4, max_iter: int=1000, alpha: float=0.5):
        
            # Rotor mass depends only on geometry, so compute it once here
            self.rotor_mass = (
                ingenuity.blade_mass
                * self.N_blades
                * self.N_rotors
                * self.rotor_diameter / ingenuity.rotor_diameter
            )
            P_drone = P_initial
            # compute the mass of one rotor (Scale linearly with rotor diameter (same mass per unit length as ingenuity))
            # self.rotor_mass = ingenuity.blade_mass * self.N_blades * self.N_rotors * self.rotor_diameter / ingenuity.rotor_diameter

            
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
                
                if not np.isfinite(P_new) or not np.isfinite(self.mass):
                    print(f"Warning: non-finite result for {self.name} at iteration {i+1}.")
                    return None

                
                # 6. Check convergence
                if abs(P_new - P_drone) < tol:
                    # print(f"Converged in {i+1} iterations")
                    return self
                
                # 7. Relaxed update — blend old and new estimate to avoid oscillation
                P_drone = alpha * P_new + (1 - alpha) * P_drone
            
            print(f"Warning: did not converge after {max_iter} iterations. Residual = {abs(P_new - P_drone):.4f} W")
            return None

