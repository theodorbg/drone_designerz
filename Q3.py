import numpy as np
from funcs import drone_power
from Q1 import P_rotors_total, R, m, c_mean

# Ingenuity reference parameters
m_ingenuity = m # kg
m_fus_ingenuity = 0.3
m_ingenuity_no_fuselage = m_ingenuity - m_fus_ingenuity # kg
R_ingenuity = R # m
P_ingenuity = P_rotors_total # W

# Fixed drone parameters
N_blade = 2 # number of blades per rotor
R_2rotor = 0.625758   # m
R_4rotor = 0.509091
rpm = 2800 # RPM's

n_batteries = 6 # 
e_battery = 10/6 # Wh per battery
max_payload = 2
m_battery = 47/1e3
max_m_battery_pack = max_payload + n_batteries * m_battery







def solve_rotor_design(R, N_blade, N_rotor, n_extra_batteries):
    
    # BATTERY PACK PARAMETERS
    m_battery_pack = (n_batteries + n_extra_batteries) * m_battery # kg
    
    # Iterative solver
    P_drone = 200.0 # initial guess [W]
    tol = 1e-4 # convergence threshold [W]
    max_iter = 1000
    alpha = 0.5 # relaxation factor
    
    # OTHER COMPONENTS
    m_components = 0.9 # kg
    m_propeller_one = 0.07/4 * N_blade * R / R_ingenuity
    m_propellers = N_rotor * m_propeller_one

    for i in range(max_iter):

        # 1. Estimate mass components that depend on P_drone
        m_motor_per_rotor = (0.25 / 2) * (P_drone / P_ingenuity) # kg
        m_motor = m_motor_per_rotor * N_rotor # kg

        # 2. Build up total mass
        m_no_fuselage = m_battery_pack + m_components + m_propellers + m_motor
        m_fuselage = m_no_fuselage * (m_fus_ingenuity / m_ingenuity_no_fuselage)    
        m_total = m_fuselage + m_no_fuselage

        # 3. Compute required power for this mass
        P_new = drone_power(m_total, R, c_mean, rpm, N_blade, N_rotor)

        # 4. Check convergence
        if abs(P_new - P_drone) < tol:
            P_drone = P_new
            print(f"Converged in {i+1} iterations")
            return P_drone, m_total
            break

        # 5. Relaxed update — blend old and new estimate to avoid oscillation
        P_drone = alpha * P_new + (1 - alpha) * P_drone
        
        # else:
        #     print(f"Warning: did not converge after {max_iter} iterations. Residual = {abs(P_new - P_drone):.4f} W")
        
# Run the solver for both configurations
# loop over a range of extra batteries to see how it affects the results for both configurations
rotor2_results = {}
rotor4_results = {}


for n_extra_batteries in range(0,42):
    print(f"\n=== Results for {n_extra_batteries} extra batteries ===")
    # add the results to a dictionary for later plotting
    P_drone_2rotor, m_total_2rotor = solve_rotor_design(R_2rotor, N_blade, 2, n_extra_batteries)
    P_drone_4rotor, m_total_4rotor = solve_rotor_design(R_4rotor, N_blade, 4, n_extra_batteries)
    
    # flight time from 20 Wh battery
    e_drone = (n_batteries + n_extra_batteries) * e_battery # total energy in Wh
    flight_time_hours_2rotor = e_drone / P_drone_2rotor
    flight_time_hours_4rotor = e_drone / P_drone_4rotor
    flight_time_minutes_2rotor = 60.0 * flight_time_hours_2rotor
    flight_time_minutes_4rotor = 60.0 * flight_time_hours_4rotor

    rotor2_results[n_extra_batteries] = (P_drone_2rotor, m_total_2rotor, flight_time_minutes_2rotor)
    rotor4_results[n_extra_batteries] = (P_drone_4rotor, m_total_4rotor, flight_time_minutes_4rotor)

# Plot results: pdrone vs n_extra_batteries for both configurations
import matplotlib.pyplot as plt
n_extra_batteries_list = list(rotor2_results.keys())
flight_time_minutes_2rotor_list = [rotor2_results[n][2] for n in n_extra_batteries_list]
flight_time_minutes_4rotor_list = [rotor4_results[n][2] for n in n_extra_batteries_list]
plt.figure(figsize=(10,6))
plt.plot(n_extra_batteries_list, flight_time_minutes_2rotor_list, label='2-Rotor', marker='o')
plt.plot(n_extra_batteries_list, flight_ftime_minutes_4rotor_list, label='4-Rotor', marker='o')
plt.xlabel('Number of Extra Batteries')
plt.ylabel('Flight Time (minutes)')
plt.title('Flight Time vs Number of Extra Batteries')
plt.legend()
plt.grid()
plt.savefig('flight_time_vs_batteries.png')
plt.close()

# the flight time of each configuration can be read from plot to be with max amount of battereis
# extract the last flight time for each configuration
max_flight_time_2rotor = flight_time_minutes_2rotor_list[-1]
max_flight_time_4rotor = flight_time_minutes_4rotor_list[-1]
print(f"Max flight time for 2-rotor configuration: {max_flight_time_2rotor:.2f} minutes")
print(f"Max flight time for 4-rotor configuration: {max_flight_time_4rotor:.2f} minutes")
# 12 minutes 53 seconds for 2 rotor
# 12 minutes 22 seconds for 4 rotor
