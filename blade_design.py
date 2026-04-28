import numpy as np
from numpy import pi, cos, sin, sqrt

from parse_txt_funcs import(read_polar_txt,
                           read_blade_geometry_nasa,
                           load_rotor_parameters
                           )

class BladeDesign:
    def __init__(self, drone, planet, polar_data="data/clf5605_us_fp_polar.txt", no_blade_elements=12):
        self.R = drone.rotor_radius
        # Cut off the blade at 80% of the radius to avoid the root region where the flow is complex and the blade may not be effective
        self.drone = drone
        r_cut_off = 0.2 * self.R
        self.r = np.linspace(r_cut_off, self.R, no_blade_elements)  # Avoid r=0 to prevent singularity
        
        self.polar = read_polar_txt(polar_data)
        self.y = self.r / self.R  # Normalized radius
        self.Cl = self.polar["Cl"]
        self.aoa_deg = self.polar["alpha"]  
        self.aoa_rad = np.radians(self.aoa_deg)
        # interp Cl vs aoa to find alpha0 (zero lift angle) and Cl_alpha (slope of Cl vs aoa)
        # the slope of the lift curve, denoted as Cl_alpha, is measured at the zero lift angle of attack. 
        self.alpha_0_deg, self.alpha_0_rad, self.Cl_alpha = self.alpha_from_cl(0.0, self.Cl, self.aoa_deg, n_fit=len(self.Cl))
        self.solidity = drone.solidity
        
        self.mass = drone.mass
        self.g = planet.g
        self.rho = planet.rho
        self.thrust = self.mass * self.g
        self.omega = drone.omega
        self.C_T = drone.C_T
        

    def compute_no_twist(self):
        
        self.no_twist = (3 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/4 * sqrt(self.C_T) ) + self.alpha_0_deg)
        
        return self
    
    def compute_linear_twist(self):
        
        theta_tip = (12/5 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/4 * sqrt(self.C_T)  + 1/3*self.alpha_0_deg))
        theta = theta_tip**(2 - self.y)
        self.linear_twist = theta
        
        return self
    
    def compute_optimum_twist(self):
        
        theta_tip = (2 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/2 * sqrt(self.C_T)  + 2/3 * self.alpha_0_deg))
        self.optimum_twist = theta_tip / self.y
        
        return self
    
    def compute_optimum_plan_form_and_twist(self, c_tip):
        
        self.chord_distribution = c_tip / self.y
        # induced_velocity = 1/2 * sqrt(self.C_T)
        # flow_angle = induced_velocity / self.y
        # alpha_D = pitch_angle - flow_angle
        self.design_aoa_deg, self.design_aoa_rad = self.find_design_aoa()
        
        theta_optimum_plan_form_rad = self.design_aoa_rad + 1/(2 * self.y) * sqrt(self.C_T)
        
        self.theta_optimum_plan_form = np.degrees(theta_optimum_plan_form_rad)
        
        return self
    
    def define_design_twist(self):
        self.design_twist = self.theta_optimum_plan_form
        #TODO DEFINE ACTUAL TWIST DISTRIBUTION
        
    def define_design_chord(self, c_tip):
        self.design_chord = self.chord_distribution
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
        aos = np.asarray(aoa)[idx]
        aoas_rad = np.radians(aos)

        # Ensure n_fit is at least 2 and does not exceed the number of available data points
        n_fit = max(2, min(n_fit, len(Cls)))

        # local linear fit near target for slope estimate: aoa = m*Cl + b
        near_idx = np.argsort(np.abs(Cls - cl_target))[:n_fit]
        near_idx = np.sort(near_idx)
        m, b = np.polyfit(Cls[near_idx], aoas_rad[near_idx], 1)

        # alpha at target Cl (interp in-range, extrap out-of-range)
        if Cls[0] <= cl_target <= Cls[-1]:
            target_aoa = np.interp(cl_target, Cls, aos)
        else:
            target_aoa = m * cl_target + b

        # Cl_alpha is dCl/d(alpha). Since m = d(alpha)/dCl, invert it.
        Cl_alpha = np.inf if np.isclose(m, 0.0) else 1.0 / m
        
        target_aoa_deg = target_aoa
        target_aoa_rad = np.radians(target_aoa_deg)

        return float(target_aoa_deg), float(target_aoa_rad), float(Cl_alpha)
    
    def prandtl_tip_correction(self, phi_rad, method="linear"):
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
     
    def bem(self, method="linear", hover=True):
        if hover==True:
            lambda_c,v_climb = 0.0
        
        sigma = self.solidity
        Cla = self.Cl_alpha_rad
        F = self.prandtl_tip_correction(phi_rad, method=method)
        theta = self.design_twist
        alpha0 = self.alpha_0_rad
        y = self.y

        # below equation 88
             
        vrel = sqrt((v_climb + u_i)**2 + (omega * r)**2) # local relative velocity
        phi_rad = np.arctan(u_i / (omega * r)) # flow angle
        
        if method == "linear":
            #eq 98
            
            if hover = = True:
                lambda_i = (sigma * Cla) / (16 * F) * (sqrt(1+32*F/(sigma * Cla) * (theta - alpha0) * y) - 1)
            else:
                lambda_i = -1/2 * (lambda_c + sigma * Cla / (8 * F) - sqrt((lambda_c + sigma * Cla / (8 * F))**2 + sigma * Cla * (theta - alpha0) * y / (2 * F) ))
