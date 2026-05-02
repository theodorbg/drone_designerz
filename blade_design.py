import numpy as np
from numpy import pi, cos, sin, sqrt

from parse_txt_funcs import(read_polar_txt,
                           read_blade_geometry_nasa,
                           load_rotor_parameters
                           )

class BladeDesign:
    def __init__(self, drone, planet, c_tip, polar_data="data/clf5605_us_fp_polar.txt", no_blade_elements=12):
        self.R = drone.rotor_radius
        self.A = pi * self.R**2
        # Cut off the blade at 80% of the radius to avoid the root region where the flow is complex and the blade may not be effective
        self.drone = drone
        r_cut_off = 0.2 * self.R
        self.r = np.linspace(r_cut_off, self.R, no_blade_elements)  # Avoid r=0 to prevent singularity
        self.c_tip = c_tip
        
        self.polar = read_polar_txt(polar_data)
        self.y = self.r / self.R  # Normalized radius
        self.Cl = self.polar["Cl"]
        self.Cd = self.polar["Cd"]
        self.aoa_deg = self.polar["alpha"]  
        self.aoa_rad = np.radians(self.aoa_deg)
        # interp Cl vs aoa to find alpha0 (zero lift angle) and Cl_alpha (slope of Cl vs aoa)
        # the slope of the lift curve, denoted as Cl_alpha, is measured at the zero lift angle of attack. 
        self.alpha_0_rad, self.Cl_alpha = self.alpha_from_cl(0.0, self.Cl, self.aoa_deg, n_fit=len(self.Cl))
        self.solidity = drone.solidity
        
        self.mass = drone.mass
        self.g = planet.g
        self.rho = planet.rho
        self.thrust = self.mass * self.g
        self.omega = drone.omega
        self.C_T = drone.C_T
        

    def compute_no_twist(self):
        
        # permanent pitch angle esentially
        self.no_twist = np.degrees((3 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/4 * sqrt(self.C_T) ) + self.alpha_0_rad))
        
        #make it into an array distributed along self.y
        self.no_twist = np.full_like(self.y, self.no_twist)
        
        return self
    
    def compute_linear_twist(self):
        
        theta_tip = (12/5 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/4 * sqrt(self.C_T)  + 1/3*self.alpha_0_rad))
        theta = theta_tip * (2 - self.y)
        self.linear_twist = np.degrees(theta)
        
        return self
    
    def compute_optimum_twist(self):
        
        theta_tip = (2 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/2 * sqrt(self.C_T)  + 2/3 * self.alpha_0_rad))
        self.optimum_twist = np.degrees(theta_tip / self.y)
        
        return self
    
    def compute_optimum_plan_form_and_twist(self):
        
        self.optimum_chord_distribution = self.c_tip / self.y
        # induced_velocity = 1/2 * sqrt(self.C_T)
        # flow_angle = induced_velocity / self.y
        # alpha_D = pitch_angle - flow_angle
        self.design_aoa_deg, self.design_aoa_rad = self.find_design_aoa()
        
        theta_optimum_plan_form_rad = self.design_aoa_rad + 1/(2 * self.y) * sqrt(self.C_T)
        
        self.theta_optimum_plan_form = np.degrees(theta_optimum_plan_form_rad)
        
        return self
    
    def define_twist(self):
        """ this function actually defines what the design twist is
        TODO: DO SOMETHING FROM THE ROOT UNTIL 0.2*R
        """
        #DEGREES
        self.twist = self.theta_optimum_plan_form
        #TODO DEFINE ACTUAL TWIST DISTRIBUTION
        
    def define_chord(self):
        """ this function actually defines what the design chord distribution is
        TODO: DO SOMETHING FROM THE ROOT UNTIL 0.2*R
        """
        self.chord = self.optimum_chord_distribution
        #TODO DEFINE ACTUAL CHORD DISTRIBUTION
    
    def find_design_aoa(self):
        # the design angle of attack is chosen at the aoa where cd/cl is at the minimum value
        cd_cl = self.polar["Cd"] / self.polar["Cl"]
        min_idx = np.argmin(cd_cl)
        design_aoa_deg = self.polar["alpha"][min_idx]
        design_aoa_rad = np.radians(design_aoa_deg)
        
        return design_aoa_deg, design_aoa_rad
    
    def alpha_from_cl(self, cl_target, Cl, aoa, n_fit=3):
        """
        Estimate the angle of attack (aoa) corresponding to a target lift coefficient (cl_target) using local linear fitting.
        This method sorts the provided Cl and aoa data, performs a local linear fit around the target Cl, 
        and then uses the fit to estimate the aoa at the target Cl.
        It also calculates the slope of the lift curve (Cl_alpha) at that point.
        
        Parameters:
        - cl_target: The target lift coefficient for which we want to find the corresponding angle of attack.
        - Cl: An array of lift coefficients corresponding to the provided angles of attack.
        - aoa: An array of angles of attack.
        - n_fit: The number of points to use for the local linear fit.
        """
        # Sort the data by Cl to ensure proper interpolation and fitting
        idx = np.argsort(Cl)
        # Reorder Cl and aoa according to the sorted indices
        Cls = np.asarray(Cl)[idx]
        # Reorder aoa according to the sorted indices
        aoas_rad = np.radians(np.asarray(aoa)[idx])
        

        # Ensure n_fit is at least 2 and does not exceed the number of available data points
        n_fit = max(2, min(n_fit, len(Cls)))

        # local linear fit near target for slope estimate: aoa = m*Cl + b
        near_idx = np.argsort(np.abs(Cls - cl_target))[:n_fit]
        near_idx = np.sort(near_idx)
        m, b = np.polyfit(Cls[near_idx], aoas_rad[near_idx], 1)

        # alpha at target Cl (interp in-range, extrap out-of-range)
        if Cls[0] <= cl_target <= Cls[-1]:
            target_aoa_rad = np.interp(cl_target, Cls, aoas_rad)
        else:
            target_aoa_rad = m * cl_target + b

        # Cl_alpha is dCl/d(alpha). Since m = d(alpha)/dCl, invert it.
        Cl_alpha = np.inf if np.isclose(m, 0.0) else 1.0 / m

        return float(target_aoa_rad), float(Cl_alpha)
    
    def prandtl_tip_correction(self, phi_rad: float, method: str):
        #eq 89
        r = self.r
        phi_deg = np.degrees(phi_rad)
        Nb = self.drone.N_blades
        R = self.R
        if method == "linear":
            sin_phi = phi_rad
        else:
            sin_phi = np.sin(phi_rad)
        
        f = Nb / 2 * (R - r) / (r * sin_phi)
        F = 2 / pi * np.arccos(np.exp(-f))
        return F

    def prandtl_tip_correction_dimensionless(self, phi: float):
        
        Nb = self.drone.N_blades
        y = self.y
        sin_phi = np.sin(phi) # phi in radians
        
        f = Nb / 2 * (1-y)/(y * sin_phi)
        
        F = 2 / pi * np.arccos(np.exp(-f))
        return F
    
    def bem(self, tol=1e-6):
        """
        Simplified vectorized BEM loop for hover.
        Requires: self.twist (degrees-like array or scalar) and self.chord (array) set before calling.
        Returns per-element induced velocity v_i, vrel, dT_be, dT_mom.
        """

        
        # C is constant hard coded to 0.5 (typical value (slide 10 lec 11))
        C = 0.5
        A = self.A
        R = self.R
        omega = self.omega
        sigma = self.solidity
        Cla = self.Cl_alpha
        theta_rad = np.radians(self.twist)
        alpha0_rad = self.alpha_0_rad
        y = self.y
        Nb = self.drone.N_blades
        c = self.chord
        rho = self.rho
        r = self.r
        if theta_rad is None or c is None:
            raise ValueError("design_twist and design_chord must be set before calling bem()")
        
        # WE ARE IN HOVER SO v_climb = 0
        v_climb = 0.0


        # below equation 88
             
        # phi_rad = np.arctan(u_i / (omega * r)) # flow angle
        #TODO FIND OUT IF IT IS THIS EQUATION OR THIS:             phi_rad = np.arccos(omega * r / vrel) # flow angle

        
        
    
        
        # assume initial induced velocity value v_i guess per element
        v_i = np.zeros_like(r)
        dr = r[1] - r[0]  # Assuming uniform spacing of blade elements
        diff = np.inf  # Initialize diff to a large value for the while loop

        while diff > tol:
            
            #relative velocity
            vrel = sqrt((v_climb + v_i)**2 + (omega * r)**2) # local relative velocity
            
            # flow angle
            # phi_rad = np.arccos(omega * r / vrel) # flow angle
            
            
            phi_rad = np.arctan2(v_climb + v_i, omega * r)  # flow angle

            # angle of attack
            aoa_rad = theta_rad - phi_rad
            # interpolate lift and drag coefficients from polar data (self.polar)
            cl = np.interp(aoa_rad, self.aoa_rad, self.Cl)
            cd = np.interp(aoa_rad, self.aoa_rad, self.Cd)
            # tip correction
            F = self.prandtl_tip_correction(phi_rad, method="non-linear")
            
            # Thrust (Blade element (BE))
            dT_be = 0.5 * Nb * rho * c * vrel**2 * (cl * cos(phi_rad) - cd * sin(phi_rad)) * dr
            # Thrust (momentum (mom))
            dT_mom = 4 * pi * rho * (v_climb + v_i) * v_i * r * dr
            # update induced velocity
            v_i = v_i + C * (dT_be - F * dT_mom)
            
            # check for convergence
            diff = float(np.max(np.abs(dT_be - F * dT_mom)))
        
        # store final values
        self.dPower = 0.5 * Nb * omega * rho * c * vrel**2 * (cl * sin(phi_rad) + cd * cos(phi_rad)) * r * dr
        self.dCt_mom = dT_mom / (0.5 * rho * A * (omega * R)**2)
        self.v_i = v_i
        self.vrel = vrel
        self.dT_be = dT_be
        self.dT_mom = dT_mom
        return self

    def bem_linear(self, tol=1e-6):
        """
        Simplified vectorized BEM loop for hover.
        Requires: self.twist (degrees-like array or scalar) and self.chord (array) set before calling.
        Returns per-element induced velocity v_i, vrel, dT_be, dT_mom.
        """        
        # C is constant hard coded to 0.5 (typical value (slide 10 lec 11))
        C = 0.5
        A = self.A
        R = self.R
        omega = self.omega
        sigma = self.solidity
        Cla = self.Cl_alpha
        theta_rad = np.radians(self.twist)
        alpha0_rad = self.alpha_0_rad
        y = self.y
        Nb = self.drone.N_blades
        c = self.chord
        rho = self.rho
        r = self.r
        if theta_rad is None or c is None:
            raise ValueError("design_twist and design_chord must be set before calling bem()")
        
        # WE ARE IN HOVER SO v_climb = 0
        v_climb = 0.0
        # below equation 88             
        # phi_rad = np.arctan(u_i / (omega * r)) # flow angle
        #TODO FIND OUT IF IT IS THIS EQUATION OR THIS:             phi_rad = np.arccos(omega * r / vrel) # flow angle

        # assume initial induced velocity value v_i guess per element
        v_i = np.zeros_like(r)
        dr = r[1] - r[0]  # Assuming uniform spacing of blade elements
        diff = np.inf  # Initialize diff to a large value for the while loop

        while diff > tol:
            
            #relative velocity
            vrel = sqrt((v_climb + v_i)**2 + (omega * r)**2) # local relative velocity
            
            # flow angle
            # phi_rad = np.arccos(omega * r / vrel) # flow angle
            
            
            phi_rad = np.arctan2(v_climb + v_i, omega * r)  # flow angle
            cos_phi = 1
            sin_phi = phi_rad

            # angle of attack
            aoa_rad = theta_rad - phi_rad
            # interpolate lift and drag coefficients from polar data (self.polar)
            cl = Cla * (aoa_rad - alpha0)  # linear lift curve assumption            cd = np.interp(aoa_rad, self.aoa_rad, self.Cd)
            # tip correction
            F = self.prandtl_tip_correction(phi_rad, method="linear")
            
            # Thrust (Blade element (BE))
            dT_be = 0.5 * Nb * rho * c * vrel**2 * (cl * cos_phi - cd * sin_phi) * dr
            # Thrust (momentum (mom))
            dT_mom = 4 * pi * rho * (v_climb + v_i) * v_i * r * dr
            # update induced velocity
            v_i = v_i + C * (dT_be - F * dT_mom)
            
            # check for convergence
            diff = float(np.max(np.abs(dT_be - F * dT_mom)))
        
        # store final values
        self.dPower = 0.5 * Nb * omega * rho * c * vrel**2 * (cl * sin_phi + cd * cos_phi) * r * dr
        self.dCt_mom = dT_mom / (0.5 * rho * A * (omega * R)**2)
        self.v_i = v_i
        self.vrel = vrel
        self.dT_be = dT_be
        self.dT_mom = dT_mom
        return self

    def bem_dimensionless(self, tol=1e-6):
        # lambda_i = v_i / (omega * r)
        # lambda_c = v_climb / (omega * r)
        
        lambda_i = np.zeros_like(self.r)
        lambda_c = np.zeros_like(self.r)
        Nb = self.drone.N_blades
        omega = self.omega
        rho = self.rho
        c = self.chord
        theta_rad = np.radians(self.twist)
        sigma = self.solidity
        y = self.y
        r = self.r
        R = self.R
        A = self.A
        
        diff = np.inf  # Initialize diff to a large value for the while loop
        dy = y[1] - y[0]  # Assuming uniform spacing of blade elements in dimensionless radius

        while diff > tol:
            phi_rad = np.arctan2(lambda_i, y)  # flow angle
            dC_T_mom = 8 * (lambda_c + lambda_i) * lambda_i * y * dy
            dC_T_be = sigma * ((lambda_c + lambda_i)**2 + y**2) * (cl*cos(phi_rad) - cd * sin(phi_rad)) * dy
            F = self.prandtl_tip_correction_dimensionless(phi_rad)
            lambda_i = lambda_i + C * (dC_T_be - F * dC_T_mom)
            diff = float(np.max(np.abs(dC_T_be - F * dC_T_mom)))
        
        self.dC_T_be = dC_T_be
        self.dC_T_mom = dC_T_mom
        v_i = lambda_i * omega * R
        aoa_rad = theta_rad - phi_rad
        # interpolate lift and drag coefficients from polar data (self.polar)
        cl = np.interp(aoa_rad, self.aoa_rad, self.Cl)
        cd = np.interp(aoa_rad, self.aoa_rad, self.Cd)

        self.dPower = 0.5 * Nb * omega * rho * c * vrel**2 * (cl * sin(phi_rad) + cd * cos(phi_rad)) * r * dr
        self.dCt_mom = dT_mom / (0.5 * rho * A * (omega * R)**2)

        self.dT_be = self.dC_T_be *(0.5 * rho * A * (omega * R)**2)

         
    