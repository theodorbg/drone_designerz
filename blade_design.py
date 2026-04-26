import numpy as np
from numpy import pi, cos, sin, sqrt

from parse_txt_funcs import(read_polar_txt,
                           read_blade_geometry_nasa,
                           load_rotor_parameters
                           )

class BladeDesign:
    def __init__(self, drone, planet, polar_data="data/clf5605_us_fp_polar.txt", no_blade_elements=12):
        self.R = drone.rotor_radius
        self.r = np.linspace(0.1 * self.R, self.R, no_blade_elements)  # Avoid r=0 to prevent singularity
        
        self.polar = read_polar_txt(polar_data)
        self.y = self.r / self.R  # Normalized radius
        self.Cl = self.polar["Cl"]
        self.aoa = self.polar["alpha"]  
        # interp Cl vs aoa to find alpha0 (zero lift angle) and Cl_alpha (slope of Cl vs aoa)
        # the slope of the lift curve, denoted as Cl_alpha, is measured at the zero lift angle of attack. 
        self.alpha_0, self.Cl_alpha = self.alpha_from_cl(0.0, self.Cl, self.aoa, n_fit=len(self.Cl))
        self.solidity = drone.solidity
        
        self.mass = drone.mass
        self.g = planet.g
        self.rho = planet.rho
        self.thrust = self.mass * self.g
        self.omega = drone.omega
        self.C_T = drone.C_T
        

    def compute_no_twist(self):
        
        self.no_twist = 3 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/4 * sqrt(self.C_T) ) + self.alpha_0 
        
        
        return self
    
    def compute_linear_twist(self):
        
        theta_tip = 12/5 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/4 * sqrt(self.C_T)  + 1/3*self.alpha_0)
        theta = theta_tip**(2 - self.y)
        self.linear_twist = theta
        
        return self
    
    def compute_optimum_twist(self):
        
        theta_tip = 2 * (self.C_T / (self.solidity * self.Cl_alpha) + 1/2 * sqrt(self.C_T)  + 2/3 * self.alpha_0)
        self.optimum_twist = theta_tip / self.y
        
        return self
    
    def compute_optimum_plan_form_and_twist(self, c_tip):
        
        self.chord_distribution = c_tip / self.y
        # induced_velocity = 1/2 * sqrt(self.C_T)
        # flow_angle = induced_velocity / self.y
        # alpha_D = pitch_angle - flow_angle
        self.design_aoa = self.find_design_aoa()
        
        self.theta_optimum_plan_form = self.design_aoa + 1/(2 * self.y) * sqrt(self.C_T)
        
        return self
    
    def find_design_aoa(self):
        # the design angle of attack is chosen at the aoa where cd/cl is at the minimum value
        cd_cl = self.polar["Cd"] / self.polar["Cl"]
        min_idx = np.argmin(cd_cl)
        self.design_aoa = self.polar["alpha"][min_idx]
        return self.design_aoa
    
    def alpha_from_cl(self, cl_target, Cl, aoa, n_fit=3):
        idx = np.argsort(Cl)
        Cls = np.asarray(Cl)[idx]
        aos = np.asarray(aoa)[idx]

        n_fit = max(2, min(n_fit, len(Cls)))

        # local linear fit near target for slope estimate: aoa = m*Cl + b
        near_idx = np.argsort(np.abs(Cls - cl_target))[:n_fit]
        near_idx = np.sort(near_idx)
        m, b = np.polyfit(Cls[near_idx], aos[near_idx], 1)

        # alpha at target Cl (interp in-range, extrap out-of-range)
        if Cls[0] <= cl_target <= Cls[-1]:
            target_aoa = np.interp(cl_target, Cls, aos)
        else:
            target_aoa = m * cl_target + b

        # Cl_alpha is dCl/d(alpha). Since m = d(alpha)/dCl, invert it.
        Cl_alpha = np.inf if np.isclose(m, 0.0) else 1.0 / m

        return float(target_aoa), float(Cl_alpha)
    
