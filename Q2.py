import numpy as np
from funcs import drone_power_stacked_rotors

# Ingenuity reference parameters
m_ingenuity = 1.8 # kg
m_fus_ingenuity = 0.3
m_ingenuity_no_fuselage = m_ingenuity - m_fus_ingenuity # kg
R_ingenuity = 0.6 # m
P_ingenuity = 177.28 # W

# Fixed drone parameters
N_blade = 2 # number of blades per rotor
N_rotor = 2 # number of rotors (stacked configuration)
R = 0.6   # m
c_mean = 0.1417 # m
rpm = 2800 # RPM's

m_payload = 0 # kg
m_battery_pack = 0.280 # kg
m_components = 0.9 # kg
m_propeller_one = 0.07/4 * N_blade * R / R_ingenuity
m_propellers = N_rotor * m_propeller_one

# Iterative solver
P_drone = 200.0 # initial guess [W]
tol = 1e-4 # convergence threshold [W]
max_iter = 1000
alpha = 0.5 # relaxation factor

for i in range(max_iter):

    # 1. Estimate mass components that depend on P_drone
    m_motor_per_rotor = (0.25 / 2) * (P_drone / P_ingenuity) # kg
    m_motor = m_motor_per_rotor * N_rotor # kg

    # 2. Build up total mass
    m_no_fuselage = m_payload + m_battery_pack + m_components + m_propellers + m_motor
    m_fuselage = m_no_fuselage * (m_fus_ingenuity / m_ingenuity_no_fuselage)    
    m_total = m_fuselage + m_no_fuselage

    # 3. Compute required power for this mass
    P_new = drone_power_stacked_rotors(m_total, R, c_mean, rpm, N_blade, N_rotor)

    # 4. Check convergence
    if abs(P_new - P_drone) < tol:
        P_drone = P_new
        print(f"Converged in {i+1} iterations")
        break

    # 5. Relaxed update — blend old and new estimate to avoid oscillation
    P_drone = alpha * P_new + (1 - alpha) * P_drone

else:
    print(f"Warning: did not converge after {max_iter} iterations. Residual = {abs(P_new - P_drone):.4f} W")

# Results
print(f"P_drone  = {P_drone:.3f} W")
print(f"m_total  = {m_total:.3f} kg")