import numpy as np
from numpy import pi, cos, sin, sqrt

from parse_txt_funcs import(read_polar_txt,
                           read_blade_geometry_nasa,
                           load_rotor_parameters
                           )

class BladeDesign:
    def __init__(self, drone, planet, c_tip, polar_data="data/clf5605_us_fp_polar.txt", no_blade_elements=30):
        
        self.polar = read_polar_txt(polar_data)
        
        self.Cl = self.polar["Cl"]
        self.Cd = self.polar["Cd"]
        self.aoa_deg = self.polar["alpha"]  
        self.aoa_rad = np.radians(self.aoa_deg)
        
        self.R = drone.rotor_radius
        self.A = pi * self.R**2
        # Cut off the blade at 80% of the radius to avoid the root region where the flow is complex and the blade may not be effective
        self.drone = drone
        r_cut_off = 0.2 * self.R
        # self.r = np.linspace(r_cut_off, self.R, no_blade_elements)
        self.r = np.linspace(r_cut_off, 0.99 * self.R, no_blade_elements) # we go from 0.2R to 0.99R to avoid complex flow near root and NaNs for Prandtl at r=R
        # self.r = np.linspace(r_cut_off, self.R, no_blade_elements)  # Avoid r=0 to prevent singularity

        self.c_tip = c_tip
        
        
        self.y = self.r / self.R  # Normalized radius
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
    
    def set_twist(self, twist_values):
        """ this function actually defines what the design twist is
        TODO: DO SOMETHING FROM THE ROOT UNTIL 0.2*R
        """
        #DEGREES
        self.twist = twist_values
        #TODO DEFINE ACTUAL TWIST DISTRIBUTION
        
    def set_chord(self, chord_values):
        """ this function actually defines what the design chord distribution is
        TODO: DO SOMETHING FROM THE ROOT UNTIL 0.2*R
        """
        self.chord = chord_values
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
    
    def prandtl_tip_correction(self, phi_rad: float, method: str, dimensionless=False):
        #eq 89
        r = self.r
        y = self.y
        phi_deg = np.degrees(phi_rad)
        Nb = self.drone.N_blades
        R = self.R
        if method == "linear":
            sin_phi = phi_rad
        if method == "non-linear":
            sin_phi = np.sin(phi_rad)
        
        sin_phi = np.maximum(np.abs(sin_phi), 1e-6)  #  prevent divide by zero

        if dimensionless == False:
            f = Nb / 2 * (R - r) / (r * sin_phi)
        if dimensionless == True:
            f = Nb / 2 * (1 - y) / (y * sin_phi)
        F = 2 / pi * np.arccos(np.exp(-f))
        F = np.clip(F, 1e-4, 1.0)   # make sure F never reaches exactly 0

        return F

    def bem(self, tol=1e-6, max_iter=500, linear=False, dimensionless=False, C=0.5, verbose=False):
        """
        Master BEM solver.

        Args:
            linear:
                True  -> linear lift model: Cl = Cla * (aoa - alpha0)
                False -> nonlinear polar interpolation
            dimensionless:
                True  -> solve in lambda_i = v_i / (Omega R)
                False -> solve in v_i [m/s]
        """
        R = self.R
        A = self.A
        omega = self.omega
        sigma = self.solidity
        Cla = self.Cl_alpha
        alpha0 = self.alpha_0_rad
        theta = np.radians(self.twist)
        y = self.y
        r = self.r
        Nb = self.drone.N_blades
        c = np.asarray(self.chord, dtype=float)
        rho = self.rho
        dy = y[1] - y[0]
        dr = r[1] - r[0]

        if theta is None or c is None:
            raise ValueError("twist and chord must be set before calling bem_master().")

        # Initial guess
        if dimensionless:
            lambda_i = np.full_like(r, 4.0 / (omega * R), dtype=float)
            lambda_c = np.zeros_like(r)
            v_climb = np.zeros_like(r)
        else:
            v_i = np.full_like(r, 4.0, dtype=float)
            v_climb = np.zeros_like(r)

        diff = np.inf
        iteration = 0

        while diff > tol and iteration < max_iter:
            if dimensionless:
                phi = np.arctan2(lambda_c + lambda_i, y)
                vrel_dl = np.sqrt((lambda_c + lambda_i) ** 2 + y ** 2)
                aoa = theta - phi
            else:
                vrel = np.sqrt((v_climb + v_i) ** 2 + (omega * r) ** 2)
                phi = np.arctan2(v_climb + v_i, omega * r)
                aoa = theta - phi

            # Aerodynamics
            if linear:
                cl = Cla * (aoa - alpha0)
                cl = np.clip(cl, np.min(self.Cl), np.max(self.Cl))
                cd = np.interp(aoa, self.aoa_rad, self.Cd)
            else:
                cl = np.interp(aoa, self.aoa_rad, self.Cl)
                cd = np.interp(aoa, self.aoa_rad, self.Cd)

            # Tip loss
            prandtl_method = "linear" if linear else "non-linear"
            F = self.prandtl_tip_correction(phi, method=prandtl_method, dimensionless=dimensionless)

            # Update induced velocity
            if dimensionless:
                dCT_be = sigma * vrel_dl**2 * (cl * np.cos(phi) - cd * np.sin(phi)) * dy
                dCT_mom = 8.0 * F * (lambda_c + lambda_i) ** 2 * y * dy

                if linear:
                    radicand = 1.0 + 32.0 * F / (sigma * Cla) * (theta - alpha0) * y
                    lambda_i_new = sigma * Cla / (16.0 * F) * (np.sqrt(np.maximum(radicand, 0.0)) - 1.0)
                else:
                    lambda_i_new = np.sqrt(
                        np.maximum(dCT_be / (8.0 * F * np.maximum(y, 1e-12) * dy), 0.0)
                    )

                diff = float(np.max(np.abs(lambda_i_new - lambda_i)))
                lambda_i = lambda_i_new if linear else 0.5 * lambda_i + 0.5 * lambda_i_new

            else:
                dT_be = 0.5 * Nb * rho * c * vrel**2 * (cl * np.cos(phi) - cd * np.sin(phi)) * dr
                dT_mom = 4.0 * np.pi * rho * (v_climb + v_i) * v_i * r * dr

                if linear:
                    radicand = 1.0 + 32.0 * F / (sigma * Cla) * (theta - alpha0) * y
                    lambda_i_new = sigma * Cla / (16.0 * F) * (np.sqrt(np.maximum(radicand, 0.0)) - 1.0)
                    v_i_new = lambda_i_new * omega * R
                    diff = float(np.max(np.abs(v_i_new - v_i)))
                    v_i = v_i_new
                else:
                    v_i_new = np.sqrt(
                        np.maximum(dT_be / (4.0 * np.pi * rho * np.maximum(F, 1e-12) * np.maximum(r, 1e-12) * dr), 0.0)
                    )
                    diff = float(np.max(np.abs(v_i_new - v_i)))
                    v_i = 0.5 * v_i + 0.5 * v_i_new

            if verbose and (iteration < 5 or iteration % 50 == 0):
                print(f"iter={iteration:4d} diff={diff:.3e} F=[{np.min(F):.3f},{np.max(F):.3f}]")

            iteration += 1

        if iteration >= max_iter and verbose:
            print("Warning: bem_master reached max_iter before convergence.")

        # Final storage
        if dimensionless:
            v_i = lambda_i * omega * R
            vrel = np.sqrt((v_climb + v_i) ** 2 + (omega * r) ** 2)
            phi = np.arctan2(v_climb + v_i, omega * r)
            aoa = theta - phi

            if linear:
                cl = Cla * (aoa - alpha0)
                cl = np.clip(cl, np.min(self.Cl), np.max(self.Cl))
                cd = np.interp(aoa, self.aoa_rad, self.Cd)
            else:
                cl = np.interp(aoa, self.aoa_rad, self.Cl)
                cd = np.interp(aoa, self.aoa_rad, self.Cd)

            dCT_be = sigma * ((lambda_i) ** 2 + y ** 2) * (cl * np.cos(phi) - cd * np.sin(phi)) * dy
            dCT_mom = 8.0 * F * (lambda_i ** 2) * y * dy

            dT_be = dCT_be * (0.5 * rho * A * (omega * R) ** 2)
            dT_mom = dCT_mom * (0.5 * rho * A * (omega * R) ** 2)
            dC_T = dCT_be
        else:
            vrel = np.sqrt((v_climb + v_i) ** 2 + (omega * r) ** 2)
            phi = np.arctan2(v_climb + v_i, omega * r)
            aoa = theta - phi

            if linear:
                cl = Cla * (aoa - alpha0)
                cl = np.clip(cl, np.min(self.Cl), np.max(self.Cl))
                cd = np.interp(aoa, self.aoa_rad, self.Cd)
            else:
                cl = np.interp(aoa, self.aoa_rad, self.Cl)
                cd = np.interp(aoa, self.aoa_rad, self.Cd)

            dT_be = 0.5 * Nb * rho * c * vrel**2 * (cl * np.cos(phi) - cd * np.sin(phi)) * dr
            dT_mom = 4.0 * np.pi * rho * (v_climb + v_i) * v_i * r * dr
            dC_T = dT_be / (0.5 * rho * A * (omega * R) ** 2)

        self.v_i = v_i
        self.vrel = vrel
        self.dT = dT_be
        self.dT_be = dT_be
        self.dT_mom = dT_mom
        self.dC_T = dC_T
        self.dPower = 0.5 * Nb * omega * rho * c * vrel**2 * (cl * np.sin(phi) + cd * np.cos(phi)) * r * dr

        return self
