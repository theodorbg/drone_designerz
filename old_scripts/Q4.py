import numpy as np
import matplotlib.pyplot as plt
from Q1 import P_rotors_total, R, m, c_mean
# from Q3 import rpm, R_2rotor, R_4rotor
rpm = 2800
R_2rotor = 0.76
R_4rotor = 0.66


rho_earth = 1.225 
visc_earth = 1.8e-5
rho_mars = 0.01503
visc_mars = 1.3e-5

l_chord = c_mean

R_2rotor_75 = 0.75 * R_2rotor
R_4rotor_75 = 0.75 * R_4rotor

V_2rotor = rpm * 2 * np.pi * R_2rotor_75 / 60.0
V_4rotor = rpm * 2 * np.pi * R_4rotor_75 / 60.0

# Calculate Reynolds numbers
Re_2rotor_earth = rho_earth * V_2rotor * l_chord / visc_earth
Re_4rotor_earth = rho_earth * V_4rotor * l_chord / visc_earth
Re_2rotor_mars = rho_mars * V_2rotor * l_chord / visc_mars
Re_4rotor_mars = rho_mars * V_4rotor * l_chord / visc_mars

# Calculate differences in Reynolds numbers
delta_Re_2rotor = Re_2rotor_earth - Re_2rotor_mars
delta_Re_4rotor = Re_4rotor_earth - Re_4rotor_mars



# print results
print(f"Reynolds number for 2-rotor configuration on Earth: {Re_2rotor_earth:.2e}")
print(f"Reynolds number for 4-rotor configuration on Earth: {Re_4rotor_earth:.2e}")
print(f"Reynolds number for 2-rotor configuration on Mars: {Re_2rotor_mars:.2e}")
print(f"Reynolds number for 4-rotor configuration on Mars: {Re_4rotor_mars:.2e}")
print(f"Difference in Reynolds number for 2-rotor configuration: {delta_Re_2rotor:.2e}")
print(f"Difference in Reynolds number for 4-rotor configuration: {delta_Re_4rotor:.2e}")

# print results as latex table with Reynolds as columsn and configurations as rows (4x3 table)
# ...existing code...

def fmt_sci(x):
    mantissa, exp = f"{x:.2e}".split("e")
    exp = int(exp)
    return rf"${mantissa} \times 10^{{{exp}}}$"

print("\\begin{table}[h]")
print("\\centering")
print("\\begin{tabular}{lccc}")
print("\\hline")
print(r"           & $Re_{earth}$ & $Re_{mars}$ & $\Delta Re$ \\")
print("\\hline")
print(
    f"Two Rotor  & {fmt_sci(Re_2rotor_earth)} & {fmt_sci(Re_2rotor_mars)} & {fmt_sci(delta_Re_2rotor)} \\\\"
)
print(
    f"Four Rotor & {fmt_sci(Re_4rotor_earth)} & {fmt_sci(Re_4rotor_mars)} & {fmt_sci(delta_Re_4rotor)} \\\\"
)
print("\\hline")
print("\\end{tabular}")
print("\\end{table}")

# ...existing code...