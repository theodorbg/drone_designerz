import numpy as np

def drone_power(m_drone, r_rotor, c_mean, rpm, N_blades, N_rotors):
    # Mars parameters
    g = 3.712 # gravitational constant [m/s^2]
    # air density (assumed 1 meter above surface) [kg/m^3]
    rho = (0.699 * np.exp(-0.00009*1))/(0.1921 * ((-31 - 0.000998 * 1) + 273.1)) 


    # Drone parameters
    A = np.pi * r_rotor**2 # swept disk area of one rotor disk
    omega = 2 * np.pi * rpm / 60  # rad/s
    C_d0 = 0.02 # zero-lift drag coefficient
    gamma = 1.15 # induced-loss correction factor

    # Total thrust in hover [m*kg/s^2]
    T_total = m_drone * g 

    # Total ideal power (slide 8 of lecture notes)
    P_total_ideal = T_total**(3/2) / np.sqrt(2 * rho * A * N_rotors)
        
    # Power loss due to drag for the two rotors (slide 13 of lecture notes)
    P0 = N_rotors * (1/8) * rho * c_mean * N_blades * C_d0 * omega**3 * r_rotor**4

    # Total power for both rotors (slide 14 of lecture notes)
    P_rotor_total = gamma * P_total_ideal + P0
    
    return P_rotor_total