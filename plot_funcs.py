import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches

def plot_big_grid_search(converged_drone_designs, N_blade_array, filename, fig_title):
    
        # # Color map for left column: colored by number of blades
        # blade_colors = {n: c for n, c in zip(N_blade_array, ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'])}

        # # Color map for right column: colored by rotor radius
        # unique_radii = sorted(set(d.rotor_radius for d in converged_drone_designs))
        # radius_cmap = plt.colormaps['tab10'].resampled(len(unique_radii))
        # radius_colors = {r: radius_cmap(i) for i, r in enumerate(unique_radii)}

        # for drone_name in ["Dual Copter", "Quad Copter"]:
        #     fig, ax = plt.subplots(3, 2, figsize=(14, 12))
        #     fig.suptitle(f'{drone_name} Design Performance', fontsize=14, fontweight='bold', y=1.06)

        #     subset = [d for d in converged_drone_designs if d.name == drone_name]

        #     for d in subset:
        #         c_blade  = blade_colors[d.N_blades]
        #         c_radius = radius_colors[d.rotor_radius]

        #         # Left column: vs rotor radius, colored by N_blades
        #         ax[0,0].scatter(d.rotor_radius, d.hover_power,      color=c_blade)
        #         ax[1,0].scatter(d.rotor_radius, d.mass,             color=c_blade)
        #         ax[2,0].scatter(d.rotor_radius, d.total_hover_time, color=c_blade)

        #         # Right column: vs number of blades, colored by radius
        #         ax[0,1].scatter(d.N_blades, d.hover_power,      color=c_radius)
        #         ax[1,1].scatter(d.N_blades, d.mass,             color=c_radius)
        #         ax[2,1].scatter(d.N_blades, d.total_hover_time, color=c_radius)

        #     # Axis labels and grid
        #     for col, xlabel in enumerate(['Rotor Radius [m]', 'Number of Blades']):
        #         ax[0,col].set_ylabel('Hover Power [W]');   ax[0,col].grid(True, alpha=0.3)
        #         ax[1,col].set_ylabel('Total Mass [kg]');   ax[1,col].grid(True, alpha=0.3)
        #         ax[2,col].set_ylabel('Flight Time [s]');   ax[2,col].set_xlabel(xlabel)
        #         ax[2,col].grid(True, alpha=0.3)

        #     # Legend LEFT column: blade count
        #     blade_handles = [
        #         mpatches.Patch(color=blade_colors[n], label=f'{n} blades')
        #         for n in N_blade_array
        #     ]

        #     # Legend RIGHT column: radius
        #     radius_handles = [
        #         mpatches.Patch(color=radius_colors[r], label=f'R={r:.2f} m')
        #         for r in unique_radii
        #     ]

        #     # Place two separate legends above each column
        #     legend_left = fig.legend(
        #         handles=blade_handles,
        #         loc='upper left',
        #         ncol=len(N_blade_array),
        #         bbox_to_anchor=(0.08, 1.02),
        #         frameon=True, fontsize=10,
        #     )
        #     fig.legend(
        #         handles=radius_handles,
        #         loc='upper right',
        #         ncol=min(5, len(unique_radii)),
        #         bbox_to_anchor=(1.0, 1.035),
        #         frameon=True, fontsize=10
        #     )
        #     fig.add_artist(legend_left)  # needed when calling fig.legend() twice

        #     plt.tight_layout()
        #     plt.savefig(f'plots/{drone_name}_performance.png', dpi=150, bbox_inches='tight')
        #     plt.close()

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
                ax[0, 0].scatter(d.rotor_radius, d.hover_power, color=c_blade)
                ax[1, 0].scatter(d.rotor_radius, d.mass, color=c_blade)
                ax[2, 0].scatter(d.rotor_radius, d.total_hover_time, color=c_blade)

                # Right column: vs number of blades, colored by radius
                ax[0, 1].scatter(d.N_blades, d.hover_power, color=c_radius)
                ax[1, 1].scatter(d.N_blades, d.mass, color=c_radius)
                ax[2, 1].scatter(d.N_blades, d.total_hover_time, color=c_radius)

            # Axis labels and grid
            for col, xlabel in enumerate(['Rotor Radius [m]', 'Number of Blades']):
                ax[0, col].set_ylabel('Hover Power [W]')
                ax[1, col].set_ylabel('Total Mass [kg]')
                ax[2, col].set_ylabel('Flight Time [s]')
                ax[2, col].set_xlabel(xlabel)

                for row in range(3):
                    ax[row, col].grid(True, alpha=0.3)

            # Legends
            blade_handles = [
                mpatches.Patch(color=blade_colors[n], label=f'{n} blades')
                for n in N_blade_array
            ]

            radius_handles = [
                mpatches.Patch(color=radius_colors[r], label=f'R={r:.2f} m')
                for r in unique_radii
            ]

            legend_left = fig.legend(
                handles=blade_handles,
                loc='upper left',
                ncol=len(N_blade_array),
                bbox_to_anchor=(0.08, 1.02),
                frameon=True,
                fontsize=10,
            )
            fig.legend(
                handles=radius_handles,
                loc='upper right',
                ncol=min(5, len(unique_radii)),
                bbox_to_anchor=(1.0, 1.035),
                frameon=True,
                fontsize=10
            )
            fig.add_artist(legend_left)

            plt.tight_layout()
            plt.savefig(filename.replace(".png", f"_{drone_name.replace(' ', '_')}.png"),
                        dpi=150, bbox_inches='tight')
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
 