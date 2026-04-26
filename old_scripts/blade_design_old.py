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
    
