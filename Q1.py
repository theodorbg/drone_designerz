import matplotlib.pyplot as plt
import numpy as np

# Mars parameters
g = 3.712 # gravitational constant [m/s^2]
# air density (assumed 1 meter above surface) [kg/m^3]
rho = (0.699 * np.exp(-0.00009*1))/(0.1921 * ((-31 - 0.000998 * 1) + 273.1)) 


# Drone parameters
m = 1.8 # mass of drone [kg]
D = 1.2 # rotor diameter [m]
R = D / 2 # rotor radius [m]
A = np.pi * R**2 # swept disk area of one rotor disk
A_blade = 0.170 # m^2 per rotor
Nb = 2 # number of blades per rotors
W_peak = 510 # peak power [W]
W_hover = 360 # hover power [W]
RPM = 2800 # [1/min]
Omega = 2 * np.pi * RPM / 60  # rad/s
C_d0 = 0.02 # zero-lift drag coefficient
gamma = 1.15 # induced-loss correction factor

# Mean chord approximation for profile power
c_mean = A_blade / (Nb * R)
print(f"Mean chord: {c_mean:.4f} m")

# Total thrust in hover [m*kg/s^2]
T_total = m * g 

# Total ideal power (slide 8 of lecture notes)
P_total_ideal = T_total**(3/2) / np.sqrt(2 * rho * A)
    
# Power loss due to drag for the two rotors (slide 13 of lecture notes)
P0 = 2 * (1/8) * rho * c_mean * Nb * C_d0 * Omega**3 * R**4

# Total power for both rotors (slide 14 of lecture notes)
P_rotors_total = gamma * P_total_ideal + P0

print(f"Mars Density: {rho:.5f} kg/m^3")
print(f"Mars Gravitational Constant: {g:.3f} m/s^2")
print(f"Mean chord: {c_mean:.4f} m")

print(f"Total ideal power: {P_total_ideal:.2f} W")
print(f"Power loss: {P0:.2f} W")
print(f"Total rotor power: {P_rotors_total:.2f} W")