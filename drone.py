import matplotlib.pyplot as plt
import numpy as np
from numpy import exp, sqrt, pi
from planet import Planet

""" 
    Data from pdf file:
   
    
        Components using battery power: rotors, computer, control motors, and heating systems
        
        Assume 2800 rpm for task 1 calculations
        Assume Cd0 = 0.02 for rotor loss calculations
            
        Evaluate dualcopter vs quadcopter drone designs
        
        Find the weight, optimum radius of propellers, number of blades and total power consumption
    
    INGENUITY REFERENCE DESIGN:
    
        WEIGHT DISTRIBUTION OF INGENUITY
        
            Two rotors = 70 g = 0.07 kg
            Battery pack = 280 g = 0.28 kg
            Propulsion and control motors = 250 g = 0.25 kg
            Fuselage = 300 g = 0.3 kg
            Computer and other components = 900 g = 0.9 kg
            Total = 1800 g = 1.8 kg
            
            weight per blade = 70 g / (2 rotors * 2 blades) 
            
            Ingenuity payload = 0 kg
        
        ROTOR DESIGN
            two contra-rotating rotors
                Rotor diameter = 1.2 m
            From Balaram et al., 2018 we get a constant chord of about 55mm = 0.055 m
                c_mean = 0.055 m
        
        ROTATIONAL SPEED
            rpm: 2800 rpm
        
        POWER CONSUMPTION
            Under peak load batteries provided: 510 W
            During hover batteries provided: 360 W
        
        BATTERY PACK
            battery pack energy = 10 Wh
            batteries per pack = 6
            
    NEW DESIGN CONSTRAINTS:
    
        New design: payload = 2 kg
        
        battery pack mass = 500 g = 0.5 kg
        battery pack energy = 20 Wh 
        
        
        
        
        
    
    For each design option (two rotors and quadcopter), compare the following in a table: 
        1. Radius of the rotor 
        2. Number of blades 
        3. Total power consumption from the rotors (total power required to produce the desired 
        thrust)  
        4. Aircrafts total mass with payload 
        5. Flight time in hover 
        
        Choose what you believe is the best design and provide the weight distribution estimation (pie chart)
        
    """


class Drone:
    """
    Drone class to encapsulate all properties and methods related to the drone design and performance evaluation.
    
    notes:
    C_D0 is assumed 0.02 everywhere (from pdf assignment)
    gamma is assumed 1.15 for now (from lecture)
    Power formulas must be revisited for a more detailed calculation
        
        
    """

    def __init__(self, name: str, rotor_diameter: float, chord: float, aux_components_mass: float,
                 N_blades: int, N_rotors: int, rpm: int, N_batteries: int,
                 C_D0: float=0.02, gamma: float=1.15):
        
        self.name = name
        
        # ROTOR PROPERTIES
        self.rotor_diameter = rotor_diameter
        self.rotor_radius = rotor_diameter / 2
        self.rotor_area = pi * self.rotor_radius**2
        self.chord = chord
        self.blade_area = chord * self.rotor_radius
        self.N_blades = N_blades
        self.N_rotors = N_rotors
        
        # BATTERY PROPERTIES
        self.N_batteries = N_batteries
        
        # ROTATIONAL PROPERTIES
        self.rpm = rpm
        self.omega = 2 * pi * rpm / 60
        
        self.aux_components_mass = aux_components_mass
        
        # AERODYNAMIC PROPERTIES
        self.C_D0 = C_D0 # zero-lift drag coefficient 
        self.gamma = gamma # induced-loss correction factor (assumed 1.15 in this course) 
        
        # ATTRIBUTES TO BE COMPUTED
        self.total_thrust = None
        self.C_T = None
        self.ideal_power = None
        self.power_loss = None
        self.solidity = None
        self.Cp = None
        self.hover_power = None
        self.total_hover_time = None
        self.mass = None
        self.battery_capacity_per_battery = None
        self.total_battery_capacity = None

    def compute_battery_capacity(self):
        raise NotImplementedError("Subclasses must implement compute_battery_capacity().")

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
    
    def compute_planet_performance(self, planet):
        g = planet.g
        rho = planet.rho
        
        self.compute_thrust(g)
        self.compute_C_T(rho)
        self.compute_ideal_power(rho)
        self.compute_power_loss(rho)
        self.compute_local_solidity()
        self.compute_Cp()
        self.compute_hover_power()
        self.compute_battery_capacity()
        self.compute_flight_time()
    
    def compute_flight_time(self):
        # flight time in seconds from battery capacity and hover power
        self.total_hover_time = self.total_battery_capacity / self.hover_power

class Ingenuity(Drone):
    """
    Ingenuity class, for the reference design of the drone
    It inherits the Drone super class, but has some values hard coded for reference
    
    """
    def __init__(
        self,
        name: str="Ingenuity",
        mass: float=1.8,
        rotor_diameter: float=1.2,
        chord: float=0.055,
        aux_components_mass: float=0.9,
        fuselage_mass: float=0.3,
        battery_total_mass: float=0.28,
        rotor_mass: float=0.07,
        motor_mass: float=0.25,
        battery_total_capacity_wh: float=10.0,
        rpm: int=2800,
        N_blades: int=2,
        N_rotors: int=2,
        N_batteries: int=6,
        peak_power: float=510.0,
        avg_power: float=360.0
    ):
        super().__init__(
            name=name,
            aux_components_mass=aux_components_mass,
            rotor_diameter=rotor_diameter,
            chord=chord,
            N_blades=N_blades,
            N_rotors=N_rotors,
            N_batteries=N_batteries,
            rpm=rpm
        )

        
        self.mass = mass
        self.fuselage_mass = fuselage_mass
        self.mass_no_fuselage = self.mass - self.fuselage_mass
        self.motor_mass = motor_mass
        self.peak_power = peak_power
        self.avg_power = avg_power

        # COMPUTE GLOBAL PROPERTIES FROM INGENUITY (reference design)        
        # calculate battery properties
        self.battery_total_mass = battery_total_mass # kg
        self.mass_per_battery = self.battery_total_mass / self.N_batteries # kg per battery
        self.battery_capacity_per_battery_wh = battery_total_capacity_wh / self.N_batteries # Wh
        self.battery_capacity_per_battery = self.battery_capacity_per_battery_wh * 3600 # J per battery
        
        # calculate blade mass
        self.rotor_mass = rotor_mass # kg total for two rotors
        self.blade_mass = self.rotor_mass / (self.N_blades * self.N_rotors) # mass of one blade
        self.BLADE_MASS_FACTOR = self.blade_mass / self.rotor_radius # mass per unit length of blade, used for scaling rotor mass with diameter
        # MULTIPLY THIS FACTOR BY N_BLADES_DESIGN * R_DESIGN TO GET THE MASS OF ONE BLADE FOR THE NEW DESIGN, THEN MULTIPLY BY N_BLADES_DESIGN * N_ROTORS_DESIGN TO GET TOTAL ROTOR MASS FOR THE DRONE DESIGN
        # THIS ASSUMES MASS SCALES LINEARLY WITH ROTOR DIAMETER
    
    def compute_battery_capacity(self):
        self.total_battery_capacity = self.N_batteries * self.battery_capacity_per_battery
                    
class DroneDesign(Drone):
    """
    Drone Design class with the properties of the drone we are designing.
    It inherits the Drone super class and adds methods for solving the mass-power relationship iteratively.
    
    notes:
    C_D0 is assumed 0.02 everywhere (from pdf assignment)
    gamma is assumed 1.15 for now (from lecture)
    Power formulas must be revisited for a more detailed calculation
        
        
    """

    def __init__(
        self,
        reference: Drone,
        name: str,
        rotor_diameter: float,
        chord: float,
        N_blades: int,
        N_rotors: int,
        rpm: int,
        N_batteries: int,
        payload_mass: float = 2.0,
        aux_components_mass: float = 1.0
    ):
        super().__init__(
            name=name,
            rotor_diameter=rotor_diameter,
            chord=chord,
            aux_components_mass=aux_components_mass,
            N_blades=N_blades,
            N_rotors=N_rotors,
            rpm=rpm,
            N_batteries=N_batteries,
        )
        
        self.reference = reference
        
        # MASS PROPERTIES
        self.payload_mass = payload_mass
        
        # ATTRIBUTES TO BE COMPUTED    
        self.battery_mass = None
        self.rotor_mass = None
        self.motor_mass = None
        self.fuselage_mass = None
        self.mass_no_fuselage = None

                
    def compute_motor_mass(self, P_drone):
        """
        The weight of the propulsion and control motors of each rotor scales linearly with the 
        power in hover of the rotor (this requires your estimation from task 1). 
        For example, if the propeller of your new design requires a power Pnew, the weight of the motor driving that 
        propeller is 
        
            Pnew = (250g/2)*Pnew/Pingenuity
        
        where 250g/2 is the weight of the propulsion motors in Ingenuity divided by its number of rotors,
        and Pingenuity is the average power consumption per rotor in Ingenuity (estimated in task 1).


        """
        
        self.motor_mass = self.N_rotors * self.reference.motor_mass * P_drone / (self.reference.N_rotors * self.reference.hover_power) # kg

    def compute_battery_capacity(self):
        self.battery_capacity_per_battery = self.reference.battery_capacity_per_battery # J per battery, same as reference design
        self.total_battery_capacity = self.N_batteries * self.battery_capacity_per_battery # J

    def compute_battery_mass(self):
        self.battery_mass = self.N_batteries * self.reference.mass_per_battery # kg, scales linearly with number of batteries, same mass per battery
            
    def compute_fuselage_mass(self):
        """
        Function to compute the mass of the fuselage based on the mass of the rest of the drone and the linear relationship with the reference design (ingenuity) 
        The battery, rotor and motor mass must be computed before this happens
        
        The weight of the fuselage scales linearly with the weight of the rest of the aircraft 
        (including the payload). 
        So the mass of the fuselage is
        
            Mfus = M_no_fuselage*(Mfus_ingenuity/M_no_fuselage_ingenuity) 

        """
        
        self.mass_no_fuselage = self.payload_mass + self.aux_components_mass + self.battery_mass + self.rotor_mass + self.motor_mass
        
        # 3. compute the mass of the fuselage, by the linear relationship with reference / ingenuity
        self.fuselage_mass = self.mass_no_fuselage * (self.reference.fuselage_mass / self.reference.mass_no_fuselage)

    def compute_rotor_mass(self):
        """
        Rotor mass scales linearly with rotor diameter, using the blade mass factor from the reference design (mass per unit length of blade)
        First we compute one blade mass based on the radius and then multiply by the number of blades and rotors to get total rotor mass for the drone design
        
        From PDF:        
        The blades of your propellers are expected to have the same average weight per unit of 
        length as those in Ingenuity.
        weight of one propeller is:
        
            weight per propeller: (70g/4) * Nblades * (R/RIngenuity). 

        """
        self.blade_mass = self.reference.BLADE_MASS_FACTOR * self.rotor_radius
        self.rotor_mass = self.blade_mass * self.N_blades * self.N_rotors

    def compute_total_mass(self, P_drone):
        
        self.compute_battery_mass()
        self.compute_rotor_mass()
        self.compute_motor_mass(P_drone)
        self.compute_fuselage_mass()
        # Add up all the masses to get total mass of the drone design
        self.mass = self.payload_mass + self.battery_mass + self.aux_components_mass + self.rotor_mass + self.motor_mass + self.fuselage_mass
                
    def solve_mass_power(self, P_initial: float, planet: 'Planet',
                         tol: float=1e-4, max_iter: int=1000, alpha: float=0.5):
        
        """
        Solve the mass-power relationship iteratively, since the power depends on the mass and the mass depends on the power.
        We start with an initial guess for the power (P_initial), compute the mass based on that power, 
        then compute the hover power based on that mass, and check for convergence. We use a relaxed update to avoid oscillation.
        """
        P_drone = P_initial
        
        for i in range(max_iter):
            # First we add up all the masses of the drone components based on the current power estimate
            self.compute_total_mass(P_drone)
            # Then we compute the hover power for this design
            self.compute_planet_performance(planet)
            # The estimate is saved in self.hover_power, so we can use that as the new estimate for power consumption
            P_new = self.hover_power
            
            # Check for non-finite results to avoid infinite loops                
            if not np.isfinite(P_new) or not np.isfinite(self.mass):
                print(f"Warning: non-finite result for {self.name} at iteration {i+1}.")
                return None

            
            # Check convergence based on the absolute difference between the new power estimate and the previous one and compare to the tolerance
            if abs(P_new - P_drone) < tol:
                # print(f"Converged in {i+1} iterations")
                return self
            
            # Relaxed update — blend old and new estimate to avoid oscillation
            P_drone = alpha * P_new + (1 - alpha) * P_drone
        
        print(f"Warning: did not converge after {max_iter} iterations. Residual = {abs(P_new - P_drone):.4f} W")
        return None
            
