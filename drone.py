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

    def __init__(self, name: str, rotor_diameter: float, chord: np.ndarray, aux_components_mass: float,
                 N_blades: int, N_rotors: int, rpm: int, N_batteries: int,
                 C_D0: float=0.02, gamma: float=1.15):
        
        self.name = name
        
        # ROTOR PROPERTIES
        self.rotor_diameter = rotor_diameter
        self.rotor_radius = rotor_diameter / 2
        self.rotor_area = pi * self.rotor_radius**2
        self.chord = np.atleast_1d(np.asarray(chord, dtype=float))
        self.blade_area = np.trapz(self.chord, 
                                np.linspace(0, self.rotor_radius, len(self.chord)))
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

    # def compute_power_loss(self, rho):
    #     # Profile power loss per rotor, then sum over rotors
    #     c = self.chord
    #     Nb = self.N_blades
    #     Cd0 = self.C_D0
    #     omega = self.omega
    #     R = self.rotor_radius

    #     p0_per_rotor = 1/8 * rho * c * Nb * Cd0 * omega**3 * R**4
    #     self.power_loss = self.N_rotors * p0_per_rotor
    
    def compute_power_loss(self, rho):
        Nb   = self.N_blades
        Cd0  = self.C_D0
        omega = self.omega
        R    = self.rotor_radius
        c    = np.atleast_1d(np.asarray(self.chord, dtype=float))

        if len(c) == 1:
            # constant chord — exact closed form
            p0_per_rotor = 1/8 * rho * c[0] * Nb * Cd0 * omega**3 * R**4
        else:
            # distributed chord — integrate numerically
            # dP0 = 1/2 * rho * Nb * c(r) * Cd0 * (omega*r)^3 * dr
            r = np.linspace(0, R, len(c))
            integrand = c * (omega * r)**3          # c(r) * (Ωr)³
            p0_per_rotor = 0.5 * rho * Nb * Cd0 * np.trapz(integrand, r)

        self.power_loss = self.N_rotors * p0_per_rotor
    
    def compute_local_solidity(self):
        Nb = self.N_blades
        R  = self.rotor_radius
        r  = np.linspace(0, R, len(self.chord))
                
        # total solidity: all blades / disk area
        self.solidity = Nb * self.blade_area / (pi * R**2)
        
        # local solidity distribution (used in BEM per-element)
        self.local_solidity = Nb * self.chord / (pi * R)
        
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
    
    def bem(self, blade, linear=False, dimensionless=False):
        """
        Compute the BEM solution for the drone design, given a blade design and the planet properties.
        This is a simplified vectorized implementation of the BEM loop for hover, which requires the twist and chord distributions to be set in the blade design before calling.
        It returns the per-element induced velocity v_i, vrel, dT_be, dT_mom.
        """

        # extract necessary properties from drone and planet
        # compute bem for one blade
        blade.bem(linear=linear, dimensionless=dimensionless)
        blade.total_thrust = np.sum(blade.dT)
        blade.total_power = np.sum(blade.dPower)

        self.total_thrust_generation = blade.total_thrust * self.N_rotors * self.N_blades
        self.total_power_generation = blade.total_power * self.N_rotors * self.N_blades
        
        c_mean = np.mean(self.chord)
        self.aspect_ratio = self.rotor_radius / c_mean
        
        return self

    def print_bem_results(self):
        print(f"Total power generated by BEM: {self.total_power_generation:.2f} W, Total thrust generated by BEM: {self.total_thrust_generation:.2f} N")
        print(f"Required hover power from mass-power solver: {self.hover_power:.2f} W, required thrust: {self.total_thrust:.2f} N\n")
        print(f"Rotor aspect ratio: {self.AR:.2f}")
        print(f"Mean chord: {np.mean(self.chord):.3f} m")
        # print(f"thrust BE: {self.total_thrust_generation_be:.2f} N")
        # print(f"thrust MOM: {self.total_thrust_generation_mom:.2f} N")

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
        self.MASS_PER_RADIUS = self.blade_mass / self.rotor_radius # mass per unit length of blade, used for scaling rotor mass with diameter
        # MULTIPLY THIS FACTOR BY N_BLADES_DESIGN * R_DESIGN TO GET THE MASS OF ONE BLADE FOR THE NEW DESIGN, THEN MULTIPLY BY N_BLADES_DESIGN * N_ROTORS_DESIGN TO GET TOTAL ROTOR MASS FOR THE DRONE DESIGN
        # THIS ASSUMES MASS SCALES LINEARLY WITH ROTOR DIAMETER
        self.MASS_PER_CHORD = self.blade_mass / np.mean(self.chord) # mass per unit chord length, used for scaling rotor mass with chord
        # ASSUME: BLADE_MASS = radius * mass_per_radius + mean_chord * mass_per_chord
        # mass_per_radius = self.BLADE_MASS_FACTOR = self.blade_mass / self.rotor_radius
        # mass_per_chord = self.BLADE_CHORD_MASS_FACTOR = self.blade_mass / np.mean(self.chord)
        # THEN: BLADE_MASS_DESIGN = radius_design * mass_per_radius + mean_chord_design * mass_per_chord
    
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
        
        self.motor_mass = self.reference.motor_mass * P_drone / (self.reference.N_rotors * self.reference.hover_power) # kg

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
        # IMPLEMENT CHORD SCALING
        # ASSUME: BLADE_MASS = radius * mass_per_radius + mean_chord * mass_per_chord
        # mass_per_radius = self.BLADE_MASS_FACTOR = self.blade_mass / self.rotor_radius
        # mass_per_chord = self.BLADE_CHORD_MASS_FACTOR = self.blade_mass / np.mean(self.chord)
        # THEN: BLADE_MASS_DESIGN = radius_design * mass_per_radius + mean_chord_design * mass_per_chord

        
        self.blade_mass = self.reference.MASS_PER_RADIUS * self.rotor_radius + self.reference.MASS_PER_CHORD * np.mean(self.chord)
        # self.blade_mass = self.reference.MASS_PER_RADIUS * self.rotor_radius
        self.rotor_mass = self.blade_mass * self.N_blades * self.N_rotors

    def compute_total_mass(self, P_drone):
        
        self.compute_battery_mass()
        self.compute_rotor_mass()
        self.compute_motor_mass(P_drone)
        self.compute_fuselage_mass()
        # Add up all the masses to get total mass of the drone design
        self.mass = self.payload_mass + self.battery_mass + self.aux_components_mass + self.rotor_mass + self.motor_mass + self.fuselage_mass
                
    def solve_mass_power(self, P_initial: float, planet: 'Planet',
                         tol: float=1, max_iter: int=10000, alpha: float=0.5):
        
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
                # print(f"Warning: non-finite result for {self.name} at iteration {i+1}.")
                return None

            
            # Check convergence based on the absolute difference between the new power estimate and the previous one and compare to the tolerance
            if abs(P_new - P_drone) < tol:
                # print(f"Converged in {i+1} iterations")
                return self
            
            # Relaxed update — blend old and new estimate to avoid oscillation
            P_drone = alpha * P_new + (1 - alpha) * P_drone
        
        print(f"Warning: did not converge after {max_iter} iterations. Residual = {abs(P_new - P_drone):.4f} W")
        return None
            
    def print_stats(self):
        print(f"Drone design: {self.name}")
        print(f"  Total mass: {self.mass:.2f} kg")
        print(f"  Total thrust required for hover: {self.total_thrust:.2f} N")
        print(f"  Total thrust generated by BEM: {self.total_thrust_generation:.2f} N")
        print(f"  Total power required for hover: {self.hover_power:.2f} W")
        print(f"  Total power generated by BEM: {self.total_power_generation:.2f} W")
        print(f"  Number of batteries: {self.N_batteries}")
        print(f"  Flight time in hover: {self.total_hover_time/60:.2f} minutes")
        print(f"  Rotor radius: {self.rotor_radius:.2f} m")
        print(f"  Mean chord: {np.mean(self.chord):.3f} m")
        print(f"  Aspect ratio: {self.aspect_ratio:.2f}")
        
    def to_latex_table(self, name: str) -> str:
        rows = [
            
            ("Total mass",                        f"{self.mass:.2f}",                          "kg"),
            ("Thrust required (hover)",           f"{self.total_thrust:.2f}",                  "N"),
            ("Thrust generated (BEM)",            f"{self.total_thrust_generation:.2f}",       "N"),
            ("Power required (hover)",            f"{self.hover_power:.2f}",                   "W"),
            ("Power generated (BEM)",             f"{self.total_power_generation:.2f}",        "W"),
            ("Number of batteries",               f"{self.N_batteries}",                       "-"),
            ("Flight time (hover)",               f"{self.total_hover_time/60:.2f}",           "min"),
            ("Rotor radius",                      f"{self.rotor_radius:.2f}",                  "m"),
            ("Mean chord",                        f"{np.mean(self.chord):.3f}",                "m"),
            ("Aspect ratio",                      f"{self.aspect_ratio:.2f}",                  "-"),
        ]

        lines = []
        lines.append(r"\begin{table}[H]")
        lines.append(r"    \centering")
        lines.append(r"    \begin{tabular}{lcc}")
        lines.append(r"        \hline")
        lines.append(r"        \textbf{Parameter} & \textbf{Value} & \textbf{Unit} \\")
        lines.append(r"        \hline")
        for label, value, unit in rows:
            lines.append(f"        {label} & {value} & {unit} \\\\")
        lines.append(r"        \hline")
        lines.append(r"    \end{tabular}")
        lines.append(f"    \\caption{{Design summary: {name}}}")
        lines.append(f"    \\label{{tab:design_{name}}}")
        lines.append(r"\end{table}")

        return "\n".join(lines)
        
class Aircraft(DroneDesign):
    """
    Aircraft class, for the final design of the drone, which is an aircraft with a certain aspect ratio and twist distribution.
    It inherits the DroneDesign class and adds methods for solving the BEM and computing the performance of the aircraft design.
    
    """
    def __init__(
        self,
        reference: Drone,
        name: str,
        rotor_diameter: float,
        chord: np.ndarray,
        N_blades: int,
        N_rotors: int,
        wingspan: float,
        wing_chord: float,
        rpm: int,
        N_batteries: int,
        payload_mass: float = 2.0,
        aux_components_mass: float = 1.0
    ):
        super().__init__(
            reference=reference,
            name=name,
            rotor_diameter=rotor_diameter,
            chord=chord,
            N_blades=N_blades,
            N_rotors=N_rotors,
            rpm=rpm,
            N_batteries=N_batteries,
            payload_mass=payload_mass,
            aux_components_mass=aux_components_mass
        )
        self.WING_DENSITY = 74 # kg/m^3
        self.C_D_BODY = 0.4 # drag coefficient of the BODY
        
        self.wingspan = wingspan
        self.wing_chord = wing_chord
    
    def solve_induced_velocity(self, V_forward: float, beta: float, planet: 'Planet', T: float,
                           tol: float = 1e-6, max_iter: int = 500) -> float:
        """
        Solve for induced velocity v_i in forward flight using Newton-Raphson.
        Based on Eq. 78 (Glauert momentum theory, Section 1.4).

        Args:
            V_forward : forward speed [m/s]
            beta  : rotor tilt angle from horizontal [rad]
            planet : planet object containing atmospheric properties
            T     : rotor thrust [N]
        Returns:
            v_i   : induced velocity at rotor disc [m/s]
        """
        rho = planet.rho
        A = self.rotor_area * self.N_rotors  # total disc area of all rotors
        V_perp = V_forward * np.sin(beta)   # component perpendicular to rotor disc
        V_par  = V_forward * np.cos(beta)   # component parallel to rotor disc
        rhs    = T / (2.0 * rho * A)    # = v_i_hover² (hover reference)

        # Initial guess: hover induced velocity (exact solution at V_forward=0)
        v_i = np.sqrt(rhs)

        for _ in range(max_iter):
            denom = np.sqrt((V_perp + v_i)**2 + V_par**2)
            f     = v_i * denom - rhs           # residual
            df    = denom + v_i * (V_perp + v_i) / denom   # df/dv_i
            
            delta = f / df
            v_i  -= delta
            v_i   = max(v_i, 1e-10)  # keep physical (positive)
            
            if abs(delta) < tol:
                break

        return v_i
    
    def solve_rotor_angle(self, V_forward: float, planet: 'Planet', 
                      wing_lift: float = 0.0, wing_drag: float = 0.0) -> tuple[float, float]:
        """ 
        Solve force balance for rotor tilt angle beta and total thrust T.
        Based on Eq. 69-70 (Section 1.4).

        Args:
            V_forward  : forward flight speed [m/s]
            planet     : planet object containing atmospheric properties
            wing_lift  : lift generated by wings [N] (0 if no wings)
            wing_drag  : drag generated by wings [N] (0 if no wings)

        Returns:
            beta : rotor tilt angle from horizontal [rad]
            T    : total rotor thrust [N]
        """
        rho = planet.rho
        W   = self.mass * planet.g

        D_body = 0.5 * rho * V_forward**2 * self.C_D_BODY * self.rotor_area * self.N_rotors

        # Wings offload some thrust and add drag
        W_eff = W - wing_lift
        D_eff = D_body + wing_drag

        beta = np.arctan2(D_eff, W_eff)       # Eq. 70
        T    = W_eff / np.cos(beta)            # Eq. 69

        # print(np.degrees(beta))
        return beta, T

    def forward_flight_power(self, V_forward: float, planet: 'Planet',
                         wing_lift: float = 0.0, wing_drag: float = 0.0) -> tuple[float, float, float]:
        """
        Compute total power required in forward flight.
        Based on Eq. 80-81 (Section 1.4).

        Args:
            V_forward  : forward flight speed [m/s]
            planet     : planet object containing atmospheric properties
            wing_lift  : lift generated by wings [N] (0 if no wings)
            wing_drag  : drag generated by wings [N] (0 if no wings)

        Returns:
            P_total : total power consumption [W]
            beta    : rotor tilt angle [rad]
            T       : total rotor thrust [N]
        """
        # Step 1: force balance -> beta and T
        beta, T = self.solve_rotor_angle(V_forward, planet, wing_lift, wing_drag)

        # Step 2: induced velocity from Glauert momentum theory
        v_i = self.solve_induced_velocity(V_forward, beta, planet, T)

        # Step 3: induced power with losses + profile drag power (Eq. 81)
        # P_induced = self.gamma * T * v_i
        # self.compute_power_loss(planet.rho)   # updates self.power_loss
        # P_profile = self.power_loss
        P_ideal = T * (V_forward * np.sin(beta) + v_i)
        P0 = 1/8 * planet.rho * self.N_blades * self.omega**3 * self.C_D0 * (np.mean(self.chord) / self.rotor_radius) * self.rotor_radius**5
        P_total = self.gamma * P_ideal + P0

        # P_total = P_induced + P_profile

        return P_total, beta, T
    
    def compute_fuselage_mass(self, wing_mass):
        """
        Function to compute the mass of the fuselage based on the mass of the rest of the drone and the linear relationship with the reference design (ingenuity) 
        The battery, rotor and motor mass must be computed before this happens
        
        The weight of the fuselage scales linearly with the weight of the rest of the aircraft 
        (including the payload). 
        So the mass of the fuselage is
        
            Mfus = M_no_fuselage*(Mfus_ingenuity/M_no_fuselage_ingenuity) 

        """
        
        self.mass_no_fuselage = self.payload_mass + self.aux_components_mass + self.battery_mass + self.rotor_mass + self.motor_mass + wing_mass
        
        # 3. compute the mass of the fuselage, by the linear relationship with reference / ingenuity
        self.fuselage_mass = self.mass_no_fuselage * (self.reference.fuselage_mass / self.reference.mass_no_fuselage)

    def compute_power_fixed_beta(self, V_forward: float, beta_deg: float, planet: 'Planet'):
        beta = np.radians(beta_deg)
        T = self.mass * planet.g / np.cos(beta)  # required thrust to balance weight at this tilt
        v_i = self.solve_induced_velocity(V_forward, beta, planet, T)
        P_ideal = T * (V_forward * np.sin(beta) + v_i)
        P0 = 1/8 * planet.rho * self.N_blades * self.omega**3 * self.C_D0 * (np.mean(self.chord) / self.rotor_radius) * self.rotor_radius**5
        P_total = self.gamma * P_ideal + P0

        return P_total, T

        
    
    def compute_total_mass(self, P_drone, wing_mass):
        
        self.compute_battery_mass()
        self.compute_rotor_mass()
        self.compute_motor_mass(P_drone)
        self.compute_fuselage_mass(wing_mass)
        # Add up all the masses to get total mass of the drone design
        self.mass = self.payload_mass + self.battery_mass + self.aux_components_mass + self.rotor_mass + self.motor_mass + self.fuselage_mass + wing_mass

    def solve_mass_power(self, P_initial: float, planet: 'Planet', wing_mass: float,
                         tol: float=1, max_iter: int=10000, alpha: float=0.5):
        
        """
        Solve the mass-power relationship iteratively, since the power depends on the mass and the mass depends on the power.
        We start with an initial guess for the power (P_initial), compute the mass based on that power, 
        then compute the hover power based on that mass, and check for convergence. We use a relaxed update to avoid oscillation.
        """
        P_drone = P_initial
        
        for i in range(max_iter):
            # First we add up all the masses of the drone components based on the current power estimate
            self.compute_total_mass(P_drone, wing_mass)
            # Then we compute the hover power for this design
            self.compute_planet_performance(planet)
            # The estimate is saved in self.hover_power, so we can use that as the new estimate for power consumption
            P_new = self.hover_power
            
            # Check for non-finite results to avoid infinite loops                
            if not np.isfinite(P_new) or not np.isfinite(self.mass):
                # print(f"Warning: non-finite result for {self.name} at iteration {i+1}.")
                return None

            
            # Check convergence based on the absolute difference between the new power estimate and the previous one and compare to the tolerance
            if abs(P_new - P_drone) < tol:
                # print(f"Converged in {i+1} iterations")
                # if self.mass < 100:
                return self
            
            # Relaxed update — blend old and new estimate to avoid oscillation
            P_drone = alpha * P_new + (1 - alpha) * P_drone
        
        print(f"Warning: did not converge after {max_iter} iterations. Residual = {abs(P_new - P_drone):.4f} W")
        return None

    def compute_reynolds(self, V_forward: float, planet: 'Planet'):
        
        
        # get mean chord
        c = np.mean(self.wing_chord)
        
        # Calculate Reynolds number
        self.Re = planet.rho * V_forward * c / planet.viscosity
        return self
