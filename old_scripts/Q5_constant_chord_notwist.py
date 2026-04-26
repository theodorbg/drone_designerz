import numpy as np
from Q1 import c_mean, g_mars, Omega
from Q2_iteration_testing import R_2_rotor_optimal, R_4_rotor_optimal
from Q4 import rho_mars
from read_polar import read_polar_txt
import matplotlib.pyplot as plt
from numpy import pi, cos, sin, sqrt

class BladeDesign:
    def __init__(self, R, N_blades, N_rotor, mass, g=g_mars, rho=rho_mars, chord=c_mean, omega=Omega, polar_data="polars/clf5605_us_fp_polar.txt", no_blade_elements=12):
        self.R = R
        self.r = np.linspace(0.2 * R, R, no_blade_elements)  # Avoid r=0 to prevent singularity
        self.N_blades = N_blades
        self.N_rotor = N_rotor
        self.chord = chord
        self.polar = read_polar_txt(polar_data)
        self.y = self.r / self.R  # Normalized radius
        self.Cl = self.polar["Cl"]
        self.aoa = self.polar["alpha"]  # Angle of attack corresponding to Cl values
        # interp Cl vs aoa to find alpha0 (zero lift angle) and Cl_alpha (slope of Cl vs aoa)
        # the slope of the lift curve, denoted as Cl_alpha, is measured at the zero lift angle of attack. 
        self.alpha_0, self.Cl_alpha = self.alpha_from_cl(0.0, self.Cl, self.aoa, n_fit=len(self.Cl))
        
        # calculate local solidity
        self.solidity = N_blades * chord / (pi * self.R) 
        
        self.mass = mass
        self.g = g
        self.rho = rho
        self.thrust = self.mass * self.g
        self.omega = omega
        self.Ct = self.thrust / (0.5 * self.rho * (pi * self.R**2) * (self.R * self.omega)**2)
        

    def compute_no_twist(self):
        
        self.no_twist = 3 * (self.Ct / (self.solidity * self.Cl_alpha) + 1/4 * sqrt(self.Ct) ) + self.alpha_0 
        
        
        return self
    
    def compute_linear_twist(self):
        
        theta_tip = 12/5 * (self.Ct / (self.solidity * self.Cl_alpha) + 1/4 * sqrt(self.Ct)  + 1/3*self.alpha_0)
        theta = theta_tip**(2 - self.y)
        self.linear_twist = theta
        
        return self
    
    def compute_optimum_twist(self):
        
        theta_tip = 2 * (self.Ct / (self.solidity * self.Cl_alpha) + 1/2 * sqrt(self.Ct)  + 2/3 * self.alpha_0)
        self.optimum_twist = theta_tip / self.y
        
        return self
    
    def compute_optimum_plan_form_and_twist(self):
        c_tip = self.chord
        self.chord_distribution = c_tip / self.y
        # induced_velocity = 1/2 * sqrt(self.Ct)
        # flow_angle = induced_velocity / self.y
        # alpha_D = pitch_angle - flow_angle
        self.design_aoa = self.find_design_aoa()
        
        self.theta_optimum_plan_form = self.design_aoa + 1/(2 * self.y) * sqrt(self.Ct)
        
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
    

# theta = 
# tsr_i

design = BladeDesign(R=R_2_rotor_optimal.R,
                     N_blades=R_2_rotor_optimal.N_blades,
                     N_rotor=R_2_rotor_optimal.N_rotor,
                     mass=R_2_rotor_optimal.m,
                     omega=Omega,
                     polar_data="polars/clf5605_us_fp_polar.txt")

design = design.compute_no_twist()

print(f"Constant chord no-twist, pitch_0 =: {design.no_twist} degrees") 

design = design.compute_linear_twist()
print(f"Constant chord linear twist, theta_tip =: {design.linear_twist[0]} degrees")
# plot the linear twist vs span
plt.figure(figsize=(8, 5))
plt.plot(design.y, design.linear_twist, marker='o')
plt.title("Linear Twist Distribution")
plt.xlabel("Normalized Span (y)")
plt.ylabel("Twist Angle (degrees)")
plt.grid()
plt.savefig("linear_twist_distribution.png")
plt.close()

design = design.compute_optimum_twist()
plt.figure(figsize=(8, 5))
plt.plot(design.y, design.optimum_twist, marker='o')
plt.title("Optimum Twist Distribution")
plt.xlabel("Normalized Span (y)")
plt.ylabel("Twist Angle (degrees)")
plt.grid()
plt.savefig("optimum_twist_distribution.png")
plt.close()
# print(design.alpha_0)  # Should print the zero lift angle of attack based on the polar data
# print(design.Cl_alpha)  # Should print the slope of the lift curve (Cl_alpha) based on the polar data
# plot Cl vs aoa
plt.figure(figsize=(8, 5))
plt.plot(design.aoa, design.Cl, marker='o')
plt.title("Cl vs Angle of Attack")
plt.xlabel("Angle of Attack (degrees)")
plt.ylabel("Cl")
plt.grid()
plt.close()

design = design.compute_optimum_plan_form_and_twist()
plt.figure(figsize=(8, 5))
plt.plot(design.y, design.theta_optimum_plan_form, marker='o')
plt.title("Optimum Planform and Twist Distribution")
plt.xlabel("Normalized Span (y)")
plt.ylabel("Twist Angle (degrees)")
plt.grid()
plt.savefig("optimum_planform_twist_distribution.png")
plt.close()

plt.figure(figsize=(8, 5))
plt.plot(design.y, design.chord_distribution, marker='o')
plt.title("Optimum chord distribution")
plt.xlabel("Normalized Span (y)")
plt.ylabel("Chord Length (m)")
plt.grid()
plt.savefig("optimum_chord_distribution.png")
plt.close()

# polar_us = read_polar_txt("polars/clf5605_us_fp_polar.txt")
# polar_jp = read_polar_txt("polars/clf5605_jp_f_polar.txt")

# print(polar_jp["alpha"])   # → array([-2. ,  0. ,  0. ,  2. ,  3. ,  4. ,  5. ,  6. ,  6. ])
# print(polar_jp["Cl"])      # → array([0.147, 0.368, 0.392, 0.540, ...])
# print(polar_jp["Cd"])      # → array([0.073, 0.067, 0.062, 0.071, ...])
# print(polar_jp["Cl_std"])  # → standard deviation of Cl measurement
# etc.