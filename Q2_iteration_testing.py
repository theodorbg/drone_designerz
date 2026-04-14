import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from funcs import drone_power_stacked_rotors

m_ingenuity = 1.8
m_fus_ingenuity = 0.3
m_ingenuity_no_fuselage = m_ingenuity - m_fus_ingenuity
R_ingenuity = 0.6

P_ingenuity_total = 177.28 # W
P_ingenuity_per_rotor = P_ingenuity_total / 2

m_payload = 2.0
m_battery_pack = 0.5
m_components = 1.0

def solve_two_rotor_design(R, N_blade, c_mean=0.1417, rpm=2800,
                           tol=1e-4, max_iter=1000, alpha=0.5):
    """
    Solve self-consistently for one 2-rotor design.
    Returns total power, total mass, motor mass, etc.
    """

    N_rotor = 4

    # mass of one propeller
    m_propeller_one = 0.07/4 * N_blade * R / R_ingenuity
    m_propellers = N_rotor * m_propeller_one

    # initial guess
    P_drone = 300.0

    converged = False

    for i in range(max_iter):
        # power per rotor for THIS design
        P_per_rotor_new = P_drone / N_rotor

        m_motor_per_rotor = (0.25 / 2) * (P_per_rotor_new / P_ingenuity_per_rotor)
        m_motor = N_rotor * m_motor_per_rotor

        # rest of aircraft
        m_no_fuselage = m_payload + m_battery_pack + m_components + m_propellers + m_motor
        m_fuselage = m_no_fuselage * (m_fus_ingenuity / m_ingenuity_no_fuselage)
        m_total = m_no_fuselage + m_fuselage

        # total power required
        P_new = drone_power_stacked_rotors(m_total, R, c_mean, rpm, N_blade, N_rotor)

        if abs(P_new - P_drone) < tol:
            P_drone = P_new
            converged = True
            break

        P_drone = alpha * P_new + (1 - alpha) * P_drone

    # flight time from 20 Wh battery
    flight_time_hours = 20.0 / P_drone
    flight_time_minutes = 60.0 * flight_time_hours

    return {
        "R": R,
        "N_blade": N_blade,
        "rpm": rpm,
        "c_mean": c_mean,
        "converged": converged,
        "iterations": i + 1,
        "m_propellers": m_propellers,
        "m_motor": m_motor,
        "m_fuselage": m_fuselage,
        "m_total": m_total,
        "P_total": P_drone,
        "flight_time_min": flight_time_minutes,
    }

R_values = np.linspace(0.35, 1.4, 100)
blade_values = [2, 3, 4, 5]

results = []

for R in R_values:
    for N_blade in blade_values:
        out = solve_two_rotor_design(R=R, N_blade=N_blade)
        results.append(out)

df_two_rotor = pd.DataFrame(results)

df_plot = df_two_rotor[df_two_rotor["N_blade"] >= 1].copy()

heatmap_data = df_plot.pivot(index="N_blade", columns="R", values="flight_time_min")
heatmap_data = heatmap_data.sort_index().sort_index(axis=1)

best_row = df_plot.loc[df_plot["flight_time_min"].idxmax()]

plt.figure(figsize=(10, 5), dpi=150)
im = plt.imshow(
    heatmap_data.values,
    aspect="auto",
    origin="lower",
    extent=[
        heatmap_data.columns.min(), heatmap_data.columns.max(),
        heatmap_data.index.min(), heatmap_data.index.max()
    ]
)

plt.scatter(best_row["R"], best_row["N_blade"], marker="x", s=100)
plt.colorbar(im, label="Flight time [min]")
plt.xlabel("Rotor radius R [m]")
plt.ylabel("Number of blades")
plt.title("Two-rotor design: flight time as function of radius and blade count")
plt.yticks(sorted(df_plot["N_blade"].unique()))
plt.show()