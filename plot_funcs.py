import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import numpy as np

def plot_big_grid_search(converged_drone_designs, N_blade_array, filename, fig_title):
    # Color map for left column: colored by number of blades
    blade_colors = {n: c for n, c in zip(N_blade_array, ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'])}

    # Color map for right column: colored by rotor radius
    unique_radii = sorted(set(d.rotor_radius for d in converged_drone_designs if d is not None))
    radius_cmap = plt.colormaps['tab10'].resampled(max(len(unique_radii), 1))
    radius_colors = {r: radius_cmap(i) for i, r in enumerate(unique_radii)}

    for drone_name in ["Dual Copter", "Quad Copter"]:
        fig, ax = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle(f'{drone_name} Design Performance', fontsize=14, fontweight='bold', y=1.06)

        subset = [d for d in converged_drone_designs if d is not None and d.name == drone_name]

        for d in subset:
            c_blade = blade_colors.get(d.N_blades, 'gray')
            c_radius = radius_colors.get(d.rotor_radius, 'gray')

            # Left column: vs rotor radius, colored by N_blades
            ax[0, 0].plot(d.rotor_radius, d.hover_power, color=c_blade, linestyle='-', marker='o', markersize=4, linewidth=1.5)
            ax[1, 0].plot(d.rotor_radius, d.mass, color=c_blade, linestyle='-', marker='o', markersize=4, linewidth=1.5)
            ax[2, 0].plot(d.rotor_radius, d.total_hover_time, color=c_blade, linestyle='-', marker='o', markersize=4, linewidth=1.5)

            # Right column: vs number of blades, colored by radius
            ax[0, 1].plot(d.N_blades, d.hover_power, color=c_radius, linestyle='-', marker='o', markersize=4, linewidth=1.5)
            ax[1, 1].plot(d.N_blades, d.mass, color=c_radius, linestyle='-', marker='o', markersize=4, linewidth=1.5)
            ax[2, 1].plot(d.N_blades, d.total_hover_time, color=c_radius, linestyle='-', marker='o', markersize=4, linewidth=1.5)

        # Axis labels and grid
        for col, xlabel in enumerate(['Rotor Radius [m]', 'Number of Blades']):
            ax[0, col].set_ylabel('Hover Power [W]')
            ax[1, col].set_ylabel('Total Mass [kg]')
            ax[2, col].set_ylabel('Flight Time [s]')
            ax[2, col].set_xlabel(xlabel)

            for row in range(3):
                ax[row, col].grid(True, alpha=0.3)

            if col == 0:
                ax[2, col].set_xticks(np.linspace(min(unique_radii), max(unique_radii), 10))
            else:
                ax[2, col].set_xticks(sorted(set(N_blade_array)))

        # Legends (left only)
        blade_handles = [
            mpatches.Patch(color=blade_colors[n], label=f"{n} blades")
            for n in N_blade_array
        ]

        fig.legend(
            handles=blade_handles,
            loc="upper left",
            ncol=len(N_blade_array),
            bbox_to_anchor=(0.08, 1.0),
            frameon=True,
            fontsize=10,
        )

        plt.tight_layout()
        plt.savefig(filename.replace(".png", f"_{drone_name.replace(' ', '_')}.png"),
                    dpi=150, bbox_inches='tight')
        plt.close()


# def plot_grid_search(converged_drone_designs, N_blade_array, filename, fig_title):
#     # Color map for left column: colored by number of blades
#     blade_colors = {n: c for n, c in zip(N_blade_array, ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'])}

#     # Color map for right column: colored by rotor radius
#     unique_radii = sorted(set(d.rotor_radius for d in converged_drone_designs if d is not None))
#     radius_cmap = plt.colormaps['tab10'].resampled(max(len(unique_radii), 1))
#     radius_colors = {r: radius_cmap(i) for i, r in enumerate(unique_radii)}

#     metrics = [
#         ("hover_power", "Hover Power [W]"),
#         ("mass", "Total Mass [kg]"),
#         ("total_hover_time", "Flight Time [s]"),
#     ]

#     for drone_name in ["Dual Copter", "Quad Copter"]:
#         fig, ax = plt.subplots(3, 2, figsize=(14, 12))
#         fig.suptitle(f"{fig_title} — {drone_name}", fontsize=14, fontweight="bold", y=1.02)

#         subset = [d for d in converged_drone_designs if d is not None and d.name == drone_name]

#         # LEFT COLUMN: x = rotor_radius, one curve per N_blades
#         for n_blades in N_blade_array:
#             series = sorted(
#                 [d for d in subset if d.N_blades == n_blades],
#                 key=lambda d: d.rotor_radius
#             )
#             if not series:
#                 continue

#             x = np.array([d.rotor_radius for d in series], dtype=float)
#             color = blade_colors.get(n_blades, "gray")

#             for row, (attr, _) in enumerate(metrics):
#                 y = np.array([getattr(d, attr) for d in series], dtype=float)
#                 ax[row, 0].plot(
#                     x, y,
#                     marker="o", linestyle="-", linewidth=1.5, markersize=4,
#                     color=color
#                 )

#         # RIGHT COLUMN: x = N_blades, one curve per rotor_radius
#         for r in unique_radii:
#             series = sorted(
#                 [d for d in subset if d.rotor_radius == r],
#                 key=lambda d: d.N_blades
#             )
#             if not series:
#                 continue

#             x = np.array([d.N_blades for d in series], dtype=float)
#             color = radius_colors.get(r, "gray")

#             for row, (attr, _) in enumerate(metrics):
#                 y = np.array([getattr(d, attr) for d in series], dtype=float)
#                 ax[row, 1].plot(
#                     x, y,
#                     marker="o", linestyle="None", markersize=4,
#                     color=color
#                 )

#         # Axis labels and grid
#         for row, (_, ylabel) in enumerate(metrics):
#             ax[row, 0].set_ylabel(ylabel)
#             ax[row, 0].grid(True, alpha=0.3)
#             ax[row, 1].grid(True, alpha=0.3)

#         ax[2, 0].set_xlabel("Rotor Radius [m]")
#         ax[2, 1].set_xlabel("Number of Blades")
#         ax[0, 0].set_title("vs Rotor Radius (color = # blades)")
#         ax[0, 1].set_title("vs Number of Blades (color = radius)")

#         if unique_radii:
#             ax[2, 0].set_xticks(np.linspace(min(unique_radii), max(unique_radii), min(10, len(unique_radii))))
#         ax[2, 1].set_xticks(sorted(set(N_blade_array)))

#         # Legends (left only)
#         blade_handles = [
#             mpatches.Patch(color=blade_colors[n], label=f"{n} blades")
#             for n in N_blade_array
#         ]

#         fig.legend(
#             handles=blade_handles,
#             loc="upper left",
#             ncol=len(N_blade_array),
#             bbox_to_anchor=(0.08, 1.0),
#             frameon=True,
#             fontsize=10,
#         )

#         plt.tight_layout()
#         plt.savefig(
#             filename.replace(".png", f"_{drone_name.replace(' ', '_')}.png"),
#             dpi=150,
#             bbox_inches="tight"
#         )
#         plt.close()

def plot_grid_search(converged_drone_designs, N_blade_array, filename, fig_title):
    # Color map: colored by number of blades
    blade_colors = {n: c for n, c in zip(N_blade_array, ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'])}

    unique_radii = sorted(set(d.rotor_radius for d in converged_drone_designs if d is not None))

    metrics = [
        ("hover_power",       "Hover Power [W]"),
        ("mass",              "Total Mass [kg]"),
        ("total_hover_time",  "Flight Time [s]"),
    ]

    drone_names = ["Dual Copter", "Quad Copter"]

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle(fig_title, fontsize=14, fontweight="bold", y=1.02)

    for col, drone_name in enumerate(drone_names):
        subset = [d for d in converged_drone_designs if d is not None and d.name == drone_name]

        for n_blades in N_blade_array:
            series = sorted(
                [d for d in subset if d.N_blades == n_blades],
                key=lambda d: d.rotor_radius
            )
            if not series:
                continue

            x = np.array([d.rotor_radius for d in series], dtype=float)
            color = blade_colors.get(n_blades, "gray")

            for row, (attr, _) in enumerate(metrics):
                y = np.array([getattr(d, attr) for d in series], dtype=float)
                axes[row, col].plot(
                    x, y,
                    marker="o", linestyle="-", linewidth=1.5, markersize=4,
                    color=color
                )

        # Axis labels and grid
        for row, (_, ylabel) in enumerate(metrics):
            axes[row, col].grid(True, alpha=0.3)
            if col == 0:
                axes[row, col].set_ylabel(ylabel)

        axes[2, col].set_xlabel("Rotor Radius [m]")
        axes[0, col].set_title(drone_name)

        if unique_radii:
            axes[2, col].set_xticks(
                np.linspace(min(unique_radii), max(unique_radii), min(10, len(unique_radii)))
            )

    # Shared legend at the top
    blade_handles = [
        mpatches.Patch(color=blade_colors[n], label=f"{n} blades")
        for n in N_blade_array
    ]
    fig.legend(
        handles=blade_handles,
        loc="upper left",
        ncol=len(N_blade_array),
        bbox_to_anchor=(0.08, 1.0),
        frameon=True,
        fontsize=10,
    )

    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close()


def plot_narrow_grid_search(converged_drone_designs, filename, fig_title):
    unique_radii = sorted(set(d.rotor_radius for d in converged_drone_designs))
    radius_cmap = plt.colormaps['tab10'].resampled(len(unique_radii))
    radius_colors = {r: radius_cmap(i) for i, r in enumerate(unique_radii)}

    fig, ax = plt.subplots(3, 2, figsize=(14, 13))
    fig.suptitle(fig_title, fontsize=14, fontweight='bold', y=0.98)

    metrics = [
        ('hover_power',      'Hover Power [W]'),
        ('mass',             'Total Mass [kg]'),
        ('total_hover_time', 'Flight Time [s]'),
    ]

    for col, drone_name in enumerate(["Dual Copter", "Quad Copter"]):
        subset = [d for d in converged_drone_designs if d.name == drone_name]

        for d in subset:
            c = radius_colors[d.rotor_radius]
            for row, (attr, ylabel) in enumerate(metrics):
                ax[row, col].scatter(d.rotor_radius, getattr(d, attr), color=c)

        for row, (_, ylabel) in enumerate(metrics):
            ax[row, col].set_xlabel('Rotor Radius [m]')
            ax[row, col].grid(True, alpha=0.3)
            if col == 0:
                ax[row, col].set_ylabel(ylabel)

        ax[0, col].set_title(drone_name, fontsize=12, fontweight='bold')

    radius_handles = [
        mpatches.Patch(color=radius_colors[r], label=f'R={r:.2f} m')
        for r in unique_radii
    ]
    fig.legend(
        handles=radius_handles,
        loc='upper center',
        ncol=len(unique_radii),
        bbox_to_anchor=(0.5, 0.96),
        frameon=True,
        fontsize=10,
        title='Rotor Radius'
    )

    plt.tight_layout()
    fig.subplots_adjust(top=0.88)
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()

def plot_weight_distribution_pie(designs, filename, titles=None):
    """
    Plot mass distribution pie charts side by side for one or more designs.
    """
    if not isinstance(designs, (list, tuple)):
        designs = [designs]

    n = len(designs)
    if titles is None:
        titles = [f"Weight Distribution — {d.name}" for d in designs]

    fig, axes = plt.subplots(1, n, figsize=(6 * n, 6))
    if n == 1:
        axes = [axes]

    for ax, design, title in zip(axes, designs, titles):
        parts = {
            "Payload": float(design.payload_mass),
            "Battery Pack": float(design.battery_mass),
            "Rotors": float(design.rotor_mass),
            "Propulsion and control motors": float(design.motor_mass),
            "Fuselage": float(design.fuselage_mass),
            "Computer and other components": float(design.aux_components_mass),
        }

        total_mass = float(design.mass)
        known_mass = sum(v for v in parts.values() if v > 0)
        other = max(total_mass - known_mass, 0.0)
        if other > 1e-9:
            parts["Other structure"] = other

        labels = [k for k, v in parts.items() if v > 1e-9]
        values = [v for v in parts.values() if v > 1e-9]

        if len(values) == 0:
            ax.set_title(f"No positive mass components for {design.name}")
            ax.axis("off")
            continue

        ax.pie(values, labels=labels, autopct="%1.1f%%", startangle=90)
        ax.set_aspect("equal")
        ax.set_title(title)

    plt.tight_layout()
    plt.savefig(filename, dpi=200, bbox_inches="tight")
    plt.close()
    
def plot_q3_battery_sweep(results, N_batt_max_2kg, filename="plots/q3_flight_time_vs_N_batteries.png"):
    """Plot flight time vs total batteries and mark unconstrained/constrained optima."""
    plt.figure(figsize=(10, 5))

    for name, color in [("Dual Copter", "tab:blue"), ("Quad Copter", "tab:orange")]:
        x = np.array(results[name]["N_batt"], dtype=int)
        y = np.array(results[name]["time_min"], dtype=float)

        if len(x) == 0:
            continue

        # Curve
        plt.plot(x, y, marker="o", linewidth=1.5, markersize=4, label=name, color=color)

        # Unconstrained optimum marker
        i_opt = int(np.argmax(y))
        x_opt = int(x[i_opt])
        y_opt = float(y[i_opt])
        plt.scatter([x_opt], [y_opt], color=color, s=60, edgecolors="black", zorder=5)

        # 2 kg-constrained optimum marker
        mask_2kg = x <= N_batt_max_2kg
        if np.any(mask_2kg):
            xc = x[mask_2kg]
            yc = y[mask_2kg]
            i_c = int(np.argmax(yc))
            x_c = int(xc[i_c])
            y_c = float(yc[i_c])
            plt.scatter([x_c], [y_c], marker="s", color=color, s=55, zorder=5)

    # Vertical line at max batteries within 2 kg
    plt.axvline(
        x=N_batt_max_2kg,
        color="red",
        linestyle="--",
        linewidth=1.5,
        label=f"2 kg limit: N_batt = {N_batt_max_2kg}"
    )

    plt.xlabel("Total number of batteries")
    plt.ylabel("Hover flight time (minutes)")
    plt.title("Q3: Flight time vs number of batteries")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=200)
    plt.close()
    
def plot_q4_polars_side_by_side(polars, filename="plots/q4_polars_side_by_side.png"):
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))

    # Left: Cl vs Cd
    ax[0].plot(polars["Cd"], polars["Cl"], marker="o", linestyle="-", markersize=3)
    ax[0].set_title("Cl vs Cd")
    ax[0].set_xlabel("Cd")
    ax[0].set_ylabel("Cl")
    ax[0].grid(True, alpha=0.3)
    ax[0].set_xlim(0.04, 0.12)
    ax[0].set_ylim(-0.2, 1.0)
    # Right: Cl vs AoA
    ax[1].plot(polars["alpha"], polars["Cl"], marker="o", linestyle="-", markersize=3)
    ax[1].set_title("Cl vs AoA")
    ax[1].set_xlabel("AoA (deg)")
    ax[1].set_ylabel("Cl")
    ax[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(filename, dpi=200)
    plt.close()

def plot_q5_twist(blade_design, blade_geometry_nasa, title="Dual Copter", filename="plots/q5_twist_distributions.png"):
    fig, ax = plt.subplots(figsize=(7, 5))

    y = blade_design.y

    ax.plot(y, blade_design.no_twist,                  marker="o", label="No twist")
    ax.plot(y, blade_design.linear_twist,              marker="o", label="Linear twist")
    ax.plot(y, blade_design.optimum_twist,             marker="o", label="Optimum twist")
    ax.plot(y, blade_design.theta_optimum_plan_form,   marker="o", label="Optimum planform + twist")
    ax.plot(blade_geometry_nasa['y'], blade_geometry_nasa['twist_deg'],
            marker='x', linestyle='--', label="NASA Reference")

    ax.set_title(title)
    ax.set_xlabel("Normalized Span (y)")
    ax.set_ylabel("Twist Angle (degrees)")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(filename, dpi=200)
    plt.close()

def plot_q5_twist_subplots(blade_designs, blade_geometry_nasa, filename="plots/q5_twist_distributions.png"):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)

    for ax, design, title in [
        (axes[0], blade_designs[0], "Dual Copter"),
        (axes[1], blade_designs[1], "Quad Copter"),
    ]:
        y = design.y

        ax.plot(y, design.no_twist, marker="o", label="No twist")
        ax.plot(y, design.linear_twist, marker="o", label="Linear twist")
        ax.plot(y, design.optimum_twist, marker="o", label="Optimum twist")
        ax.plot(y, design.theta_optimum_plan_form, marker="o", label="Optimum planform + twist")
        ax.plot(blade_geometry_nasa['y'], blade_geometry_nasa['twist_deg'], marker='x', label="NASA Reference", linestyle='--')

        ax.set_title(title)
        ax.set_xlabel("Normalized Span (y)")
        ax.grid(True)

    axes[0].set_ylabel("Twist Angle (degrees)")
    axes[1].legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=200)
    plt.close()

def plot_q5_chord_distribution(blade_design, blade_geometry_nasa, filename="plots/q5_chord_distribution.png"):
    plt.figure(figsize=(8, 5))
    plt.plot(blade_design.y, blade_design.chord, marker='o', label="Blade Design, tip chord = {:.4f} m".format(blade_design.chord[-1]))
    plt.plot(blade_geometry_nasa['y'], blade_geometry_nasa['chord'], marker='x', label="NASA Reference")
    plt.title("Optimum chord distribution")
    plt.xlabel("Normalized Span (y)")
    plt.ylabel("Chord Length (m)")
    plt.grid()
    plt.legend()
    plt.savefig(filename, dpi=200)
    plt.close()

def plot_q5_bem_chord_sweep(drone_designs, c_tip_array, thrust_margin, power_margin, filename="plots/q5_bem_chord_sweep.png"):
    """
    Plot BEM thrust/power vs required thrust/power across the tip chord sweep, along with margins.

    Assumes drone_designs is a list of DroneDesign objects corresponding to each c_tip in c_tip_array, and that each DroneDesign has attributes:
    - total_thrust_generation (from BEM)
    - total_power_generation (from BEM)
    - total_thrust (required from mass-power solver)
    - hover_power (required from mass-power solver)
    thrust and power margin: numbers representing the desired margin (e.g. 0.1 for 10% margin) to plot as dashed lines above the required values.
    """
    import matplotlib.pyplot as plt

    thrust_be  = [d.total_thrust_generation for d in drone_designs]
    power_be   = [d.total_power_generation  for d in drone_designs]
    req_thrust = [d.total_thrust            for d in drone_designs]
    req_power  = [d.hover_power             for d in drone_designs]
    thrust_margins = [thrust * (1 + thrust_margin) for thrust in req_thrust]
    power_margins = [power * (1 + power_margin) for power in req_power]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)

    # Thrust
    ax1.plot(c_tip_array, thrust_be,  'o-', label='BEM Thrust')
    ax1.plot(c_tip_array, req_thrust, 'r--', label='Required Thrust')
    ax1.plot(c_tip_array, thrust_margins, 'g--', label='Thrust Margin')
    ax1.set_ylabel("Thrust (N)")
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.5)

    # Power
    ax2.plot(c_tip_array, power_be,  'o-', color='tab:orange', label='BEM Power')
    ax2.plot(c_tip_array, req_power, 'r--', label='Required Power')
    ax2.plot(c_tip_array, power_margins, 'g--', label='Power Margin')
    ax2.set_ylabel("Power (W)")
    ax2.set_xlabel("Tip Chord (m)")
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.5)

    fig.suptitle("BEM Output vs Tip Chord (optimum twist/chord design)")
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()

def plot_q5_bem_chord_sweep_double(dual_designs, quad_designs, c_tip_array, thrust_margin, power_margin, filename="plots/q5_bem_chord_sweep_double.png"):
    """
    Plot BEM thrust/power for dual and quad copters side by side.
    
    dual_designs, quad_designs: lists of DroneDesign objects (one per c_tip)
    c_tip_array: array of tip chord values
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 10), sharex="col")

    for col, (designs, drone_name) in enumerate([(dual_designs, "Dual Copter"), (quad_designs, "Quad Copter")]):
        thrust_be = [d.total_thrust_generation for d in designs]
        power_be = [d.total_power_generation for d in designs]
        req_thrust = [d.total_thrust for d in designs]
        req_power = [d.hover_power for d in designs]
        thrust_margins = [t * (1 + thrust_margin) for t in req_thrust]
        power_margins = [p * (1 + power_margin) for p in req_power]

        ax_thrust = axes[0, col]
        ax_power = axes[1, col]

        ax_thrust.plot(c_tip_array, thrust_be, 'o-', label='BEM Thrust', color='tab:blue')
        ax_thrust.plot(c_tip_array, req_thrust, 'r--', label='Required Thrust')
        ax_thrust.plot(c_tip_array, thrust_margins, 'g--', label='Thrust Margin')
        ax_thrust.set_ylabel("Thrust (N)")
        ax_thrust.set_title(f"{drone_name} - Thrust")
        ax_thrust.legend()
        ax_thrust.grid(True, linestyle='--', alpha=0.5)

        ax_power.plot(c_tip_array, power_be, 'o-', color='tab:orange', label='BEM Power')
        ax_power.plot(c_tip_array, req_power, 'r--', label='Required Power')
        ax_power.plot(c_tip_array, power_margins, 'g--', label='Power Margin')
        ax_power.set_ylabel("Power (W)")
        ax_power.set_xlabel("Tip Chord (m)")
        ax_power.set_title(f"{drone_name} - Power")
        ax_power.legend()
        ax_power.grid(True, linestyle='--', alpha=0.5)

    fig.suptitle("BEM Output vs Tip Chord (optimum twist/chord design)")
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()


def plot_q5_dCT_dP_distribution_1x2(dual_blade, quad_blade, filename="plots/q5_dCT_dP_distribution.png"):
    """
    Plot dCT and dP distributions vs non-dimensional radius for dual and quad copter blades.
    
    2x2 layout: thrust (top), power (bottom); dual (left), quad (right)
    
    Assumes blade objects have:
    - y: non-dimensional radius (0 to 1)
    - dCT: thrust coefficient distribution
    - dPower: power distribution
    """
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), sharex="col")
    
    blades = [dual_blade, quad_blade]
    names = ["Dual Copter", "Quad Copter"]
    
    for col, (blade, name) in enumerate(zip(blades, names)):
        # Top row: dCT
        ax_thrust = axes[0, col]
        ax_thrust.plot(blade.y, blade.dC_T, 'o-', color='tab:blue', linewidth=1.5, markersize=4)
        ax_thrust.set_ylabel("dCT")
        ax_thrust.set_title(f"{name} — Thrust Coefficient Distribution")
        ax_thrust.grid(True, alpha=0.3)
        
        # Bottom row: dPower
        ax_power = axes[1, col]
        ax_power.plot(blade.y, blade.dPower, 'o-', color='tab:orange', linewidth=1.5, markersize=4)
        ax_power.set_ylabel("dP (W)")
        ax_power.set_xlabel("Non-dimensional Radius (r/R)")
        ax_power.set_title(f"{name} — Power Distribution")
        ax_power.grid(True, alpha=0.3)
    
    fig.suptitle("Blade Element Distributions (dCT and dP vs r/R)", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close()

def plot_q5_dCT_dP_distribution(blade, title="Dual Copter", filename="plots/q5_dCT_dP_distribution.png"):
    fig, axes = plt.subplots(2, 1, figsize=(7, 8), sharex=True)

    axes[0].plot(blade.y, blade.dC_T, 'o-', color='tab:blue', linewidth=1.5, markersize=4)
    axes[0].set_ylabel("dCT")
    axes[0].set_title(f"{title} — Thrust Coefficient Distribution")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(blade.y, blade.dPower, 'o-', color='tab:orange', linewidth=1.5, markersize=4)
    axes[1].set_ylabel("dP (W)")
    axes[1].set_xlabel("Non-dimensional Radius (r/R)")
    axes[1].set_title(f"{title} — Power Distribution")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Blade Element Distributions (dCT and dP vs r/R)", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close()

def plot_q5_dCT_dP_distribution_comparison(blades: list, labels: list[str], title="Dual Copter", filename="plots/q5_dCT_dP_distribution.png"):
    fig, axes = plt.subplots(2, 1, figsize=(7, 8), sharex=True)

    for blade, label in zip(blades, labels):
        axes[0].plot(blade.y, blade.dC_T,   'o-', linewidth=1.5, markersize=4, label=label)
        axes[1].plot(blade.y, blade.dPower,  'o-', linewidth=1.5, markersize=4, label=label)

    axes[0].set_ylabel("dCT")
    axes[0].set_title(f"{title} — Thrust Coefficient Distribution")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel("dP (W)")
    axes[1].set_xlabel("Non-dimensional Radius (r/R)")
    axes[1].set_title(f"{title} — Power Distribution")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Blade Element Distributions (dCT and dP vs r/R)", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close()

def plot_master_loop_2x2(designs, filename: str = "plots/master_loop_analysis_2x2.png"):

    radii_arr = np.array([d.radius for d in designs])
    cmean_arr = np.array([d.c_mean for d in designs])
    AR_arr    = np.array([d.aspect_ratio for d in designs])
    T_gen_arr = np.array([d.total_thrust_generation for d in designs])
    T_req_arr = np.array([d.total_thrust_required for d in designs])
    P_gen_arr = np.array([d.total_power_generation for d in designs])
    P_req_arr = np.array([d.total_power_required for d in designs])

    norm = mcolors.Normalize(vmin=AR_arr.min(), vmax=AR_arr.max())
    cmap = cm.plasma

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.subplots_adjust(right=0.82, wspace=0.28, hspace=0.35)

    # (ax, x_arr, x_label, y_gen_arr, y_req_arr, y_label, title)
    panels = [
        (axes[0, 0], radii_arr, "Rotor Radius R (m)",         T_gen_arr, T_req_arr, "Thrust (N)",  "Thrust vs Radius"),
        (axes[0, 1], cmean_arr, r"Mean Chord $\bar{c}$ (m)",  T_gen_arr, T_req_arr, "Thrust (N)",  "Thrust vs Mean Chord"),
        (axes[1, 0], radii_arr, "Rotor Radius R (m)",         P_gen_arr, P_req_arr, "Power (W)",   "Power vs Radius"),
        (axes[1, 1], cmean_arr, r"Mean Chord $\bar{c}$ (m)",  P_gen_arr, P_req_arr, "Power (W)",   "Power vs Mean Chord"),
    ]

    for ax, x_arr, xlabel, y_gen_arr, y_req_arr, ylabel, title in panels:
        ax.scatter(x_arr, y_gen_arr, c=AR_arr, cmap=cmap, norm=norm,
                   s=50, alpha=0.8, edgecolors='white', linewidths=0.3, zorder=3,
                   label="Generated")
        ax.scatter(x_arr, y_req_arr,
                   s=30, marker='x', color='crimson', alpha=0.7, zorder=4,
                   label="Required")
        ax.set_xlabel(xlabel, fontsize=12)
        ax.set_ylabel(ylabel, fontsize=12)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.25)

    # shared colorbar
    cbar_ax = fig.add_axes([0.84, 0.12, 0.022, 0.72])
    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cb = fig.colorbar(sm, cax=cbar_ax)
    cb.set_label(r"Aspect Ratio  AR = R / $\bar{c}$", fontsize=11)
    cb.ax.tick_params(labelsize=10)

    fig.suptitle("Master Loop — Thrust & Power Design Space", fontsize=14, fontweight='bold', y=1.01)

    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {filename}")

def plot_master_loop_3x2(designs, main_title: str, filename: str = "plots/master_loop_analysis_3x2.png"):

    radii_arr   = np.array([d.radius         for d in designs])
    cmean_arr   = np.array([d.c_mean          for d in designs])
    AR_arr      = np.array([d.aspect_ratio    for d in designs])
    T_gen_arr   = np.array([d.total_thrust_generation for d in designs])
    T_req_arr   = np.array([d.total_thrust_required   for d in designs])
    P_gen_arr   = np.array([d.total_power_generation  for d in designs])
    P_req_arr   = np.array([d.total_power_required    for d in designs])
    ft_arr      = np.array([d.flight_time     for d in designs])
    N_batt_arr  = np.array([d.N_batteries     for d in designs])

    norm = mcolors.Normalize(vmin=AR_arr.min(), vmax=AR_arr.max())
    cmap = cm.plasma

    fig, axes = plt.subplots(3, 2, figsize=(13, 13))
    fig.subplots_adjust(right=0.82, wspace=0.28, hspace=0.38)

    # (ax, x_arr, xlabel, y_gen, y_req, ylabel, title)
    panels = [
        # row 1 — thrust
        (axes[0, 0], radii_arr, "Rotor Radius R (m)",        T_gen_arr, T_req_arr, "Thrust (N)",       "Thrust vs Radius"),
        (axes[0, 1], cmean_arr, r"Mean Chord $\bar{c}$ (m)", T_gen_arr, T_req_arr, "Thrust (N)",       "Thrust vs Mean Chord"),
        # row 2 — power
        (axes[1, 0], radii_arr, "Rotor Radius R (m)",        P_gen_arr, P_req_arr, "Power (W)",        "Power vs Radius"),
        (axes[1, 1], cmean_arr, r"Mean Chord $\bar{c}$ (m)", P_gen_arr, P_req_arr, "Power (W)",        "Power vs Mean Chord"),
        # row 3 — flight time (no required line, colour still AR)
        (axes[2, 0], radii_arr, "Rotor Radius R (m)",        ft_arr,    None,      "Flight Time (s)",  "Flight Time vs Radius"),
        (axes[2, 1], cmean_arr, r"Mean Chord $\bar{c}$ (m)", ft_arr,    None,      "Flight Time (s)",  "Flight Time vs Mean Chord"),
    ]

    for ax, x_arr, xlabel, y_gen_arr, y_req_arr, ylabel, title in panels:
        sc = ax.scatter(x_arr, y_gen_arr, c=AR_arr, cmap=cmap, norm=norm,
                        s=50, alpha=0.8, edgecolors='white', linewidths=0.3, zorder=3,
                        label="Generated")

        if y_req_arr is not None:
            ax.scatter(x_arr, y_req_arr,
                       s=30, marker='x', color='crimson', alpha=0.7, zorder=4,
                       label="Required")
            ax.legend(fontsize=10)

        ax.set_xlabel(xlabel, fontsize=12)
        ax.set_ylabel(ylabel, fontsize=12)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.grid(True, alpha=0.25)

    # shared colorbar for AR
    cbar_ax = fig.add_axes([0.84, 0.12, 0.022, 0.72])
    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cb = fig.colorbar(sm, cax=cbar_ax)
    cb.set_label(r"Aspect Ratio  AR = R / $\bar{c}$", fontsize=11)
    cb.ax.tick_params(labelsize=10)

    fig.suptitle(f"{main_title} — Thrust, Power & Flight Time",
                 fontsize=14, fontweight='bold', y=1.01)

    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {filename}")

def plot_q6_wingspan_sweep(wingspan_array, powers, weights, V_forward,
                           filename="plots/q6_wingspan_sweep.png"):
    powers  = np.array(powers)
    weights = np.array(weights)

    # Find optimum
    opt_idx      = np.argmin(powers)
    opt_wingspan = wingspan_array[opt_idx]
    opt_power    = powers[opt_idx]
    opt_weight   = weights[opt_idx]

    # Baseline (no wings, b=0)
    baseline_power = powers[0]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # --- Power vs wingspan ---
    ax1.plot(wingspan_array, powers, color="tab:blue", linewidth=2)
    ax1.axhline(baseline_power, color="gray", linestyle="--", linewidth=1,
                label=f"No-wing baseline ({baseline_power:.1f} W)")
    ax1.axvline(opt_wingspan, color="tab:red", linestyle=":", linewidth=1.5,
                label=f"Optimum b = {opt_wingspan:.2f} m ({opt_power:.1f} W)")
    ax1.scatter([opt_wingspan], [opt_power], color="tab:red", zorder=5)
    ax1.set_xlabel("Wingspan $b$ [m]")
    ax1.set_ylabel("Total power [W]")
    ax1.set_title(f"Forward flight power vs. wingspan ($V$ = {V_forward} m/s)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # --- Weight vs wingspan ---
    ax2.plot(wingspan_array, weights, color="tab:green", linewidth=2)
    ax2.axvline(opt_wingspan, color="tab:red", linestyle=":", linewidth=1.5,
                label=f"Optimum b = {opt_wingspan:.2f} m ({opt_weight:.2f} kg)")
    ax2.scatter([opt_wingspan], [opt_weight], color="tab:red", zorder=5)
    ax2.set_xlabel("Wingspan $b$ [m]")
    ax2.set_ylabel("Total aircraft mass [kg]")
    ax2.set_title("Aircraft mass vs. wingspan")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()

    print(f"Optimal wingspan: {opt_wingspan:.3f} m")
    print(f"Power at optimum: {opt_power:.2f} W  (baseline: {baseline_power:.2f} W, "
          f"reduction: {(baseline_power - opt_power)/baseline_power*100:.1f}%)")
    print(f"Mass at optimum:  {opt_weight:.3f} kg")

def plot_q6_forward_flight(V_array, powers, betas, filename="plots/q6_forward_flight.png"):
        """
        Plot total power and rotor tilt angle vs forward flight speed.
        No-wing baseline case.
        """
        betas_deg = np.degrees(betas)

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

        # --- Power vs speed ---
        ax1.plot(V_array, powers, color="tab:blue", linewidth=2)
        ax1.axvline(x=10, color="gray", linestyle="--", linewidth=1, label="Design point (10 m/s)")
        ax1.set_xlabel("Forward speed [m/s]")
        ax1.set_ylabel("Total power [W]")
        ax1.set_title("Forward flight power (no wings)")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # --- Beta vs speed ---
        ax2.plot(V_array, betas_deg, color="tab:orange", linewidth=2)
        ax2.axvline(x=10, color="gray", linestyle="--", linewidth=1, label="Design point (10 m/s)")
        ax2.set_xlabel("Forward speed [m/s]")
        ax2.set_ylabel(r"Rotor tilt angle $\beta$ [deg]")
        ax2.set_title(r"Rotor tilt angle $\beta$ (no wings)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(filename, dpi=150)
        plt.close()
    
def plot_q5_master_loop_summary(best_designs, dual_designs, c_tip_array, filename="plots/q5_master_loop_summary.png"):
    import matplotlib.pyplot as plt
    import numpy as np

    # Iteration history from the master loop
    iter_x = [d.rotor_radius for d in best_designs if d is not None]
    iter_time = [d.total_hover_time / 60.0 for d in best_designs if d is not None]
    iter_power = [d.hover_power for d in best_designs if d is not None]

    # Final Q5 sweep data
    thrust_be = [d.total_thrust_generation for d in dual_designs]
    power_be = [d.total_power_generation for d in dual_designs]
    req_thrust = [d.total_thrust for d in dual_designs]
    req_power = [d.hover_power for d in dual_designs]

    fig, ax = plt.subplots(2, 2, figsize=(13, 10))

    # Top-left: flight time vs rotor radius
    ax[0, 0].plot(iter_x, iter_time, "o", linewidth=1.5, markersize=4)
    ax[0, 0].set_title("Flight Time vs Rotor Radius")
    ax[0, 0].set_xlabel("Rotor Radius [m]")
    ax[0, 0].set_ylabel("Flight Time [min]")
    ax[0, 0].grid(True, alpha=0.3)

    # Bottom-left: hover power vs rotor radius
    ax[1, 0].plot(iter_x, iter_power, "o", linewidth=1.5, markersize=4, color="tab:orange")
    ax[1, 0].set_title("Hover Power vs Rotor Radius")
    ax[1, 0].set_xlabel("Rotor Radius [m]")
    ax[1, 0].set_ylabel("Hover Power [W]")
    ax[1, 0].grid(True, alpha=0.3)

    # Top-right: thrust vs tip chord
    ax[0, 1].plot(c_tip_array, thrust_be, "o", linewidth=1.5, markersize=4, label="BEM Thrust")
    ax[0, 1].plot(c_tip_array, req_thrust, "r--", linewidth=1.2, label="Required Thrust")
    ax[0, 1].set_title("Thrust vs Tip Chord")
    ax[0, 1].set_xlabel("Tip Chord [m]")
    ax[0, 1].set_ylabel("Thrust [N]")
    ax[0, 1].grid(True, alpha=0.3)
    ax[0, 1].legend()

    # Bottom-right: power vs tip chord
    ax[1, 1].plot(c_tip_array, power_be, "o", linewidth=1.5, markersize=4, label="BEM Power", color="tab:orange")
    ax[1, 1].plot(c_tip_array, req_power, "r--", linewidth=1.2, label="Required Power")
    ax[1, 1].set_title("Power vs Tip Chord")
    ax[1, 1].set_xlabel("Tip Chord [m]")
    ax[1, 1].set_ylabel("Power [W]")
    ax[1, 1].grid(True, alpha=0.3)
    ax[1, 1].legend()

    fig.suptitle("Master Loop Summary", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close()