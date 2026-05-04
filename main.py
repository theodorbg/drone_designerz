import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import numpy as np
from planet import Planet
from drone import Drone, Ingenuity, DroneDesign
from blade_design import BladeDesign
from plot_funcs import (plot_big_grid_search,
                        plot_grid_search,
                        plot_narrow_grid_search,
                        plot_q3_battery_sweep,
                        plot_weight_distribution_pie,
                        plot_q4_polars_side_by_side,
                        plot_q5_twist_subplots,
                        plot_q5_chord_distribution,
                        plot_q5_bem_chord_sweep,
                        plot_q5_bem_chord_sweep_double,
                        plot_q5_dCT_dP_distribution,
                        plot_q5_master_loop_summary
                        )
from print_funcs import (print_q3_optimums,
                         print_q2_comparison_latex,
                         print_q4_reynolds_latex,
                         print_drone_stats
                        )
from parse_txt_funcs import(read_polar_txt,
                           read_blade_geometry_nasa,
                           load_rotor_parameters
                           )

 

#TODO DO THE FINAL GRID SEARCH

do = {
    "Q1": True,
    "Q2_BIG_GRID_SEARCH": False,
    "Q2_NARROW_GRID_SEARCH": False,
    "Q2_FINAL_GRID_SEARCH": False,
    "Q2": True,
    "Q3": True,
    "Q4": True,
    "twist_chord_single_blade": False,
    "Q5": True,
    "bem_template": False,
    "bem": False,
    "bem_master": False,
    "Q6": False,
    "MASTER_LOOP": True
    }

if do["Q1"]:
    print("\n################### Q1 ###################\n")
    print("\nRunning Q1 and generating performance metrics for Ingenuity reference design on planet: Mars\n")
    
    mars = Planet("Mars", g=3.712)
    
    # For Q1 we assume constant chord, so a square blade
    # Therefore, from nasa report we use the blade area and the rotor radius to get the chord length
    rotor_params_nasa = load_rotor_parameters("data/ingenuity_nasa_rotor_parameters.txt")
    # print(rotor_params_nasa["rotor_radius_R"])
    C_MEAN = rotor_params_nasa["blade_area"] / (rotor_params_nasa["rotor_radius_R"])
    print(f"Calculated mean chord length from NASA data with rotor radius: {C_MEAN:.4f} m")
    # We can also use the chord distribution from nasa
    # load reference data from ingenuity nasa report for twist distribution and chord distribution    
    blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
    C_MEAN = np.mean(blade_geometry_nasa["chord"])  # Alternatively, we could just take the mean chord length from the NASA data
    print(f"Calculated mean chord length from NASA data with chord distribution: {C_MEAN:.4f} m")
    C_TIP = blade_geometry_nasa['chord'][-13]  # Assuming the first entry corresponds to the tip chord
    print(f"Using tip chord length from NASA data for blade design: {C_TIP:.4f} m")
    # for our design, lets make a scaling factor to set the tip chord based on the mean chord
    #c_tip_design = c_mean_design * (C_TIP / C_MEAN)

    
    # using the blade area we get 0.28m, using the average we get 0.07m
    # Probably safe to say blade area formula is wrong, lets use the average chord in Q1-Q4 and then in Q5 we can use the actual chord distribution from the NASA data to design the blades more accurately.
    
    # lets try with the actual chord distribution
    CHORD_REF = np.array(blade_geometry_nasa["chord"])
    
    ingenuity = Ingenuity(
        name="Ingenuity",
        mass=1.8,
        rotor_diameter=1.2,
        chord=CHORD_REF,
        aux_components_mass=0.9,
        fuselage_mass=0.3,
        battery_total_mass=0.28,
        rotor_mass=0.07,
        motor_mass=0.25,
        battery_total_capacity_wh=10.0,
        rpm=2800,
        N_blades=2,
        N_rotors=2,
        N_batteries=6,
        peak_power=510.0,
        avg_power=360.0
    )

    ingenuity.compute_planet_performance(mars)

    print(f"\n{ingenuity.name}: Total Power = {ingenuity.hover_power:.2f} W, Total Mass = {ingenuity.mass:.2f} kg, Flight Time = {ingenuity.total_hover_time:.2f} s")
    #TODO LOOK INTO FORMULAS (SEE Q1_class.py)
    print("\nQ1 DONE \n")

if do["Q2_BIG_GRID_SEARCH"]:
    print("\n################### Q2_BIG_GRID_SEARCH ###################\n")
    print("\nRunning Q2 BIG GRID SEARCH\n")

    # ROTOR DIAMETER GRID SEARCH ARRAY
    # From 1/2 ingenuity to 4x ingenuity as a starting point
    diameter_array = np.linspace(ingenuity.rotor_diameter / 10, # lower bound
                                 2 * ingenuity.rotor_diameter, # upper bound
                                 10) # increments
    
    # NUMBER OF BLADES TO TEST
    N_blade_array = [2, 3, 4, 5]
    
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    # Initial guess for solver
    P_initial = ingenuity.hover_power  

    # HARDCODED CHORD FROM Q5
    chord_opt_q5 = np.array([1.37878788, 1.21350071, 1.08360023, 0.97882126, 0.89251894, 0.82020202,
                             0.75872578, 0.70582257, 0.65981598, 0.61943995, 0.58372042, 0.55189577,
                             0.52336189, 0.49763346, 0.47431611, 0.4530861,  0.43367515, 0.41585906,
                             0.39944904, 0.38428494, 0.37023008, 0.35716703, 0.34499438, 0.3336241,
                             0.32297939, 0.31299294, 0.30360553, 0.29476482, 0.28642442, 0.27854301])

    for N_blades in N_blade_array:
        for diameter in diameter_array:
        
            # NEW DRONE DESIGN
            dualCopter = DroneDesign(
                reference=ingenuity,
                name="Dual Copter",
                rotor_diameter=diameter,
                chord=chord_opt_q5,
                N_blades=N_blades,
                N_rotors=2,
                rpm=2800,
                N_batteries=6,
                payload_mass=2.0,
                aux_components_mass=1.0
            )
            
            quadCopter = DroneDesign(
                reference=ingenuity,
                name="Quad Copter",
                rotor_diameter=diameter,
                chord=chord_opt_q5,
                N_blades=N_blades,
                N_rotors=4,
                rpm=2800,
                N_batteries=6,
                payload_mass=2.0,
                aux_components_mass=1.0
            )
            
            drone_designs = [dualCopter, quadCopter]
            
            for drone in drone_designs:
                # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
                result = drone.solve_mass_power(P_initial, mars)
                if result is not None:
                    # Store the design in the list
                    converged_drone_designs.append(result)
                    # Print results for each converged design
                    # print(f"\n{drone.name}: Total Power = {drone.hover_power:.2f} W, Total Mass = {drone.mass:.2f} kg, Flight Time = {drone.total_hover_time:.2f} s")

                else:
                    print(f"Skipping {drone.name} at R={drone.rotor_radius:.2f} m, blades={drone.N_blades}")
                
                


    plot_big_grid_search(
        converged_drone_designs=converged_drone_designs,
        N_blade_array=N_blade_array,
        filename='plots/big_grid_search_performance.png',
        fig_title='Big Grid Search'
    )

        
    
    
    
        
    
    print("\nQ2 BIG GRID SEARCH DONE \n")

if do["Q2_NARROW_GRID_SEARCH"]:
    print("\n################### Q2_NARROW_GRID_SEARCH ###################\n")
    print("\nRunning Q2 NARROW GRID SEARCH\n")

    
    # ROTOR DIAMETER GRID SEARCH ARRAY
    # Now that we have already made a big grid search, we can narrow down the range of rotor diameters to test based on the results of the big grid search.
    # It was very clear that 2 blades was the best option, so we will only test 2 blades in the narrow grid search.
    # Diameter wise, 
    ROTOR_LOWER_BOUND = 0.4*2
    ROTOR_UPPER_BOUND = 1.0*2
    diameter_array = np.linspace(ROTOR_LOWER_BOUND, ROTOR_UPPER_BOUND, 10)
    
    # ROTOR_LOWER_BOUND_QUAD = 0.5*2
    # ROTOR_UPPER_BOUND_QUAD = 1.5*2
    # diameter_array_quad = np.linspace(ROTOR_LOWER_BOUND_QUAD, ROTOR_UPPER_BOUND_QUAD, 10)
    
    # NUMBER OF BLADES TO TEST
    N_blade_optimum = 2
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    # Initial guess for solver
    P_initial = ingenuity.hover_power  

    
    for diameter in diameter_array:
    
        # NEW DRONE DESIGN
        dualCopter = DroneDesign(
                reference=ingenuity,
                name="Dual Copter",
                rotor_diameter=diameter,
                chord=chord_opt_q5,
                N_blades=N_blade_optimum,
                N_rotors=2,
                rpm=2800,
                N_batteries=6,
                payload_mass=2.0,
                aux_components_mass=1.0
            )
        
        quadCopter = DroneDesign(
                reference=ingenuity,
                name="Quad Copter",
                rotor_diameter=diameter,
                chord=chord_opt_q5,
                N_blades=N_blade_optimum,
                N_rotors=4,
                rpm=2800,
                N_batteries=6,
                payload_mass=2.0,
                aux_components_mass=1.0
            )
        
        drone_designs = [dualCopter, quadCopter]
        
        for drone in drone_designs:
            # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
            result = drone.solve_mass_power(P_initial, mars)
            if result is not None:
                # Store the design in the list
                converged_drone_designs.append(result)
                # Print results for each converged design
                # print(f"\n{drone.name}: Total Power = {drone.hover_power:.2f} W, Total Mass = {drone.mass:.2f} kg, Flight Time = {drone.total_hover_time:.2f} s")

            else:
                print(f"Skipping {drone.name} at R={drone.rotor_radius:.2f} m, blades={drone.N_blades}")

    plot_narrow_grid_search(
        converged_drone_designs=converged_drone_designs,
        filename='plots/narrow_grid_search_performance.png',
        fig_title='Narrow Grid Search — 2 Blades'
    )

    
    print("\nQ2 NARROW GRID SEARCH DONE \n")

if do["Q2_FINAL_GRID_SEARCH"]:
    print("\n################### Q2_FINAL_GRID_SEARCH ###################\n")
    print("\nRunning Q2 FINAL GRID SEARCH\n")

    # HARDCODE BOUNDS BASED ON PLOTS
    ROTOR_LOWER_BOUND_DUAL = 0.6*2
    ROTOR_UPPER_BOUND_DUAL = 0.8*2
    diameter_array_dual = np.linspace(ROTOR_LOWER_BOUND_DUAL, ROTOR_UPPER_BOUND_DUAL, 20)
    
    ROTOR_LOWER_BOUND_QUAD = 0.5*2
    ROTOR_UPPER_BOUND_QUAD = 0.7*2
    diameter_array_quad = np.linspace(ROTOR_LOWER_BOUND_QUAD, ROTOR_UPPER_BOUND_QUAD, 20)
    
    # NUMBER OF BLADES TO TEST
    N_blade_optimum = 2
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    # Initial guess for solver
    P_initial = ingenuity.hover_power  

    
    for diameter in diameter_array_dual:
        dualCopter = DroneDesign(
                reference=ingenuity,
                name="Dual Copter",
                rotor_diameter=diameter,
                chord=chord_opt_q5,
                N_blades=N_blade_optimum,
                N_rotors=2,
                rpm=2800,
                N_batteries=6,
                payload_mass=2.0,
                aux_components_mass=1.0
            )
        result = dualCopter.solve_mass_power(P_initial, mars)
        if result is not None:
            # Store the design in the list
            converged_drone_designs.append(result)
            # Print results for each converged design
            # print(f"\n{dualCopter.name}: Total Power = {dualCopter.hover_power:.2f} W, Total Mass = {dualCopter.mass:.2f} kg, Flight Time = {dualCopter.total_hover_time:.2f} s")
        else:
            print(f"Skipping {dualCopter.name} at R={dualCopter.rotor_radius:.2f} m, blades={dualCopter.N_blades}")


    for diameter in diameter_array_quad:        
        quadCopter = DroneDesign(
                reference=ingenuity,
                name="Quad Copter",
                rotor_diameter=diameter,
                chord=chord_opt_q5,
                N_blades=N_blade_optimum,
                N_rotors=4,
                rpm=2800,
                N_batteries=6,
                payload_mass=2.0,
                aux_components_mass=1.0
            )
                
        # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
        result = quadCopter.solve_mass_power(P_initial, mars)
        if result is not None:
            # Store the design in the list
            converged_drone_designs.append(result)
            # Print results for each converged design
            # print(f"\n{quadCopter.name}: Total Power = {quadCopter.hover_power:.2f} W, Total Mass = {quadCopter.mass:.2f} kg, Flight Time = {quadCopter.total_hover_time:.2f} s")

        else:
            print(f"Skipping {quadCopter.name} at R={quadCopter.rotor_radius:.2f} m, blades={quadCopter.N_blades}")

    plot_narrow_grid_search(
        converged_drone_designs=converged_drone_designs,
        filename='plots/final_grid_search_performance.png',
        fig_title='Final Grid Search — 2 Blades'
    )
    
    # Choose best design based on flight time in hover (since we want to maximize flight time for a given payload)
    # choose best dualcopter and best quadcopter design
    best_dual = max([d for d in converged_drone_designs if d.name == "Dual Copter"], key=lambda d: d.total_hover_time)
    best_quad = max([d for d in converged_drone_designs if d.name == "Quad Copter"], key=lambda d: d.total_hover_time)
    
    # print results for best designs
    print(f"\nBest Dual Copter: Rotor Radius = {best_dual.rotor_radius:.2f} m, Blades = {best_dual.N_blades}, Flight Time = {best_dual.total_hover_time:.2f} s")
    print(f"Best Quad Copter: Rotor Radius = {best_quad.rotor_radius:.2f} m, Blades = {best_quad.N_blades}, Flight Time = {best_quad.total_hover_time:.2f} s")
    
    print_q2_comparison_latex(
        best_dual=best_dual,
        best_quad=best_quad,
        caption="Final Q2 comparison of dual- and quad-copter designs",
        label="tab:q2_final_comparison"
    )

    # Choose best overall by hover time and plot its weight distribution
    best_overall = best_dual if best_dual.total_hover_time >= best_quad.total_hover_time else best_quad
    print(f"Chosen best design: {best_overall.name}")

    plot_weight_distribution_pie(
        design=best_dual,
        filename="plots/q2_weight_distribution_best_dual_design.png",
        title=f"Weight Distribution — {best_dual.name}"
    )
    
    plot_weight_distribution_pie(
        design=best_quad,
        filename="plots/q2_weight_distribution_best_quad_design.png",
        title=f"Weight Distribution — {best_quad.name}"
    )

    
    print("\nQ2 FINAL GRID SEARCH DONE \n")

if do["Q2"]:
    print("\n################### Q2 SINGLE GRID SEARCH ###################\n")
    
    def grid_search_q2(diameters: np.ndarray, N_blades: np.ndarray, chord: np.ndarray, planet: Planet, reference_drone: Drone):
        
        # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
        converged_drone_designs = []
        
        # Initial guess for solver
        P_initial = ingenuity.hover_power  
        
        for diameter in diameters:
            for N_blade in N_blades:
                # NEW DRONE DESIGN
                dualCopter = DroneDesign(
                    reference=ingenuity,
                    name="Dual Copter",
                    rotor_diameter=diameter,
                    chord=chord,
                    N_blades=N_blade,
                    N_rotors=2,
                    rpm=2800,
                    N_batteries=6,
                    payload_mass=2.0,
                    aux_components_mass=1.0
                )
                
                quadCopter = DroneDesign(
                    reference=ingenuity,
                    name="Quad Copter",
                    rotor_diameter=diameter,
                    chord=chord,
                    N_blades=N_blade,
                    N_rotors=4,
                    rpm=2800,
                    N_batteries=6,
                    payload_mass=2.0,
                    aux_components_mass=1.0
                )
                
                drone_designs = [dualCopter, quadCopter]
                
                for drone in drone_designs:
                    # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
                    result = drone.solve_mass_power(P_initial, mars)
                    if result is not None:
                        # Store the design in the list
                        converged_drone_designs.append(result)
                    else:
                        print(f"Skipping {drone.name} at R={drone.rotor_radius:.2f} m, blades={drone.N_blades}")
                

            
        # Choose best design based on flight time in hover (since we want to maximize flight time for a given payload)
        # choose best dualcopter and best quadcopter design
        dual_candidates = [d for d in converged_drone_designs if d.name == "Dual Copter"]
        quad_candidates = [d for d in converged_drone_designs if d.name == "Quad Copter"]

        best_dual = max(dual_candidates, key=lambda d: d.total_hover_time) if dual_candidates else None
        best_quad = max(quad_candidates, key=lambda d: d.total_hover_time) if quad_candidates else None        
        # print results for best designs
        if best_dual is not None:
            print(f"\nBest Dual Copter: Rotor Radius = {best_dual.rotor_radius:.2f} m, Blades = {best_dual.N_blades}, Flight Time = {best_dual.total_hover_time:.2f} s")
        else:
            print("\nNo converged Dual Copter designs found.")

        if best_quad is not None:
            print(f"Best Quad Copter: Rotor Radius = {best_quad.rotor_radius:.2f} m, Blades = {best_quad.N_blades}, Flight Time = {best_quad.total_hover_time:.2f} s")
        else:
            print("No converged Quad Copter designs found.")
        
        return best_dual, best_quad, converged_drone_designs
    
    # HARDCODED CHORD FROM Q5
    chord_opt_q5 = np.array([1.37878788, 1.21350071, 1.08360023, 0.97882126, 0.89251894, 0.82020202,
                             0.75872578, 0.70582257, 0.65981598, 0.61943995, 0.58372042, 0.55189577,
                             0.52336189, 0.49763346, 0.47431611, 0.4530861,  0.43367515, 0.41585906,
                             0.39944904, 0.38428494, 0.37023008, 0.35716703, 0.34499438, 0.3336241,
                             0.32297939, 0.31299294, 0.30360553, 0.29476482, 0.28642442, 0.27854301])
    n_iter = 100
    # N_blades_array = np.array([2, 3, 4, 5])
    N_blades_array = np.array([2, 3])
    min_radius = 0.15
    max_radius = 0.8
    diameter_array = np.linspace(min_radius*2, max_radius*2, n_iter)
                
    dualCopter, quadCopter, converged_drone_designs = grid_search_q2(diameters=diameter_array, N_blades=N_blades_array, chord=chord_opt_q5, planet=mars, reference_drone=ingenuity)
    
    plot_grid_search(
        converged_drone_designs=converged_drone_designs,
        N_blade_array=N_blades_array,
        filename='plots/grid_search_q2.png',
        fig_title='Grid Search'
    )
    
    print_q2_comparison_latex(
        best_dual=dualCopter,
        best_quad=quadCopter,
        caption="Comparison of dual- and quad-copter designs",
        label="tab:q2_comparison"
    )

    plot_weight_distribution_pie(
        designs=[dualCopter, quadCopter],
        filename="plots/q2_weight_distribution_dual_quad.png",
        titles=[
            f"Weight Distribution — {dualCopter.name}",
            f"Weight Distribution — {quadCopter.name}",
        ],
    )

    
    print("\nQ2 GRID SEARCH DONE \n")

if do["Q3"]:
    print("\n################### Q3 ###################\n")
    print("\nRunning Q3 and determining optimum number of batteries\n")

    def battery_sweep_q3(dualCopter: Drone, quadCopter: Drone, planet: Planet, chord: np.ndarray):
        max_extra_payload = 2.0  # kg
        mass_per_battery = float(ingenuity.mass_per_battery)
        base_batteries = int(ingenuity.N_batteries)

        # Max extra batteries allowed by 2 kg payload constraint
        N_extra_max_2kg = int(np.floor(max_extra_payload / mass_per_battery))
        N_batt_max_2kg = base_batteries + N_extra_max_2kg

        print(f"\nMass per battery: {mass_per_battery:.4f} kg")
        print(f"Base batteries: {base_batteries}")
        print(f"Max extra batteries within 2 kg: {N_extra_max_2kg}")
        print(f"Max total batteries within 2 kg: {N_batt_max_2kg}")

        # Evaluate beyond the 2 kg limit so unconstrained optimum can be found
        N_batt_eval = np.arange(base_batteries, base_batteries + 2 * N_extra_max_2kg + 1, dtype=int)

        P_initial = ingenuity.hover_power

        results = {
            "Dual Copter": {"N_batt": [], "time_min": []},
            "Quad Copter": {"N_batt": [], "time_min": []},
        }

        for n_total in N_batt_eval:
            dual_test = DroneDesign(
                reference=ingenuity,
                name="Dual Copter",
                rotor_diameter=dualCopter.rotor_diameter,
                chord=chord,
                N_blades=dualCopter.N_blades,
                N_rotors=dualCopter.N_rotors,
                rpm=dualCopter.rpm,
                N_batteries=int(n_total),
                payload_mass=0,
                aux_components_mass=1.0
            )

            quad_test = DroneDesign(
                reference=ingenuity,
                name="Quad Copter",
                rotor_diameter=quadCopter.rotor_diameter,
                chord=chord,
                N_blades=quadCopter.N_blades,
                N_rotors=quadCopter.N_rotors,
                rpm=quadCopter.rpm,
                N_batteries=int(n_total),
                payload_mass=0,
                aux_components_mass=1.0
            )

            for drone in (dual_test, quad_test):
                result = drone.solve_mass_power(P_initial, mars)
                if result is not None:
                    results[drone.name]["N_batt"].append(int(n_total))
                    results[drone.name]["time_min"].append(drone.total_hover_time / 60.0)

        # Extract optimal battery settings from results (no new drone objects)
        optimal = {}
        for name in ["Dual Copter", "Quad Copter"]:
            x = np.array(results[name]["N_batt"], dtype=int)
            y = np.array(results[name]["time_min"], dtype=float)

            if len(x) == 0:
                optimal[name] = None
                continue

            i_opt = int(np.argmax(y))
            N_opt = int(x[i_opt])
            t_opt = float(y[i_opt])

            mask_2kg = x <= N_batt_max_2kg
            if np.any(mask_2kg):
                xc = x[mask_2kg]
                yc = y[mask_2kg]
                i_c = int(np.argmax(yc))
                N_2kg = int(xc[i_c])
                t_2kg = float(yc[i_c])
            else:
                N_2kg = None
                t_2kg = None

            optimal[name] = {
                "N_opt": N_opt,
                "t_opt": t_opt,
                "N_2kg": N_2kg,
                "t_2kg": t_2kg,
            }
            
        
        # Extract optimal drones with optimal battery counts
        optimal_dual = DroneDesign(
            reference=ingenuity,
            name="Dual Copter",
            rotor_diameter=dualCopter.rotor_diameter,
            chord=chord_opt_q5,
            N_blades=dualCopter.N_blades,
            N_rotors=dualCopter.N_rotors,
            rpm=dualCopter.rpm,
            N_batteries=optimal["Dual Copter"]["N_2kg"],
            payload_mass=0,
            aux_components_mass=1.0
        )
        optimal_dual.solve_mass_power(ingenuity.hover_power, mars)

        optimal_quad = DroneDesign(
            reference=ingenuity,
            name="Quad Copter",
            rotor_diameter=quadCopter.rotor_diameter,
            chord=chord_opt_q5,
            N_blades=quadCopter.N_blades,
            N_rotors=quadCopter.N_rotors,
            rpm=quadCopter.rpm,
            N_batteries=optimal["Quad Copter"]["N_2kg"],
            payload_mass=0,
            aux_components_mass=1.0
        )
        optimal_quad.solve_mass_power(ingenuity.hover_power, mars)
        


        return optimal_dual, optimal_quad, results, N_batt_max_2kg

    
    dualCopter, quadCopter, results, N_batt_max_2kg = battery_sweep_q3(
        dualCopter=dualCopter,
        quadCopter=quadCopter,
        planet=mars,
        chord=chord_opt_q5
    )
    
    plot_q3_battery_sweep(
        results=results,
        N_batt_max_2kg=N_batt_max_2kg,
        filename="plots/q3_flight_time_vs_N_batteries.png"
    )
    
    print_q3_optimums(results, N_batt_max_2kg)

    
    

    print("\nQ3 DONE\n")

if do["Q4"]:
    print("\n################### Q4 ###################\n")

    """
    • Cl vs Cd and Cl vs AoA at the corresponding Reynolds number.  
    Hint 1: Check the bibliography, such as: Theory of Wing Sections, Including a Summary of Airfoil 
    Data [4], and Airfoil Design and Data [5]. Airfoil tools website (http://airfoiltools.com/) 
    Hint 2: In case the source of the airfoil you chose does not have the data at the desired Reynolds, 
    import the geometry and simulate it in Xfoil. However, keep in mind that you may have to increase 
    the number of iterations and use a conservative estimate
    """
    def q4_reynolds(dualCopter: Drone, quadCopter: Drone):
        print("\nRunning Q4 and calculating Reynolds numbers for Earth and Mars\n")
        
        earth = Planet("Earth", g=9.81, rho=1.225, viscosity=1.8e-5)
        mars = Planet("Mars", g=3.72, rho=0.01503, viscosity=1.3e-5)
        
        def compute_reynolds(planet, drone, r_fraction):
            # Calculate velocity at r_fraction of rotor radius
            R = drone.rotor_radius
            
            rpm = drone.rpm
            r = r_fraction * R
            # Create radial positions from root to tip (0 to R)
            r_positions = np.linspace(0, R, len(drone.chord))
            
            # Interpolate chord at the desired radial position
            c = np.interp(r, r_positions, drone.chord)
            
            print(f"Radial position: r={r:.4f} m ({r_fraction}R), Chord: {c:.4f} m")
            
            V = rpm * 2 * np.pi * r / 60.0
            
            # Calculate Reynolds number
            Re = planet.rho * V * c / planet.viscosity
            return Re
        
        planets = [earth, mars]
        drones = [dualCopter, quadCopter]
        r_fraction = 0.75
        reynolds_results = {}
        for planet in planets:
            print(f"\nReynolds numbers at r={r_fraction}R on {planet.name}:")
            for drone in drones:
                Re = compute_reynolds(planet, drone, r_fraction)
                reynolds_results[(planet.name, drone.name)] = Re
                print(f"  {drone.name}: Re = {Re:.2e}")
                
        # Calculate differences in Reynolds numbers between Earth and Mars for each drone
        print(f"\nDifference in Reynolds numbers between Earth and Mars at r={r_fraction}R:")
        reynolds_diffs = {}
        for drone in drones:
            Re_earth = reynolds_results[("Earth", drone.name)]
            Re_mars = reynolds_results[("Mars", drone.name)]
            diff = Re_earth - Re_mars
            reynolds_diffs[drone.name] = diff
            print(f"  {drone.name}: ΔRe = {diff:.2e}")
            
        print_q4_reynolds_latex(reynolds_results, reynolds_diffs)
        
        # plot cl vs cd and cl vs aoa
        # xfoil is bugging so right now we use experimental data
        polars = read_polar_txt("data/clf5605_us_fp_polar.txt")
        plot_q4_polars_side_by_side(polars=polars, filename="plots/q4_cl_cd_and_cl_aoa.png")
        
        return polars
    
    polars = q4_reynolds(dualCopter=dualCopter, quadCopter=quadCopter)

    
    
    print("\nQ4 DONE\n")
    
if do["twist_chord_single_blade"]:
    print("\n################### Q5 ###################\n")

    print("\nRunning Q5 and designing blades based on optimum design\n")
    
    # load reference data from ingenuity nasa report for twist distribution and chord distribution    
    blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
    
    # We can use the tip chord length from the reference data to more accurately compute the chord distribution
    C_TIP = blade_geometry_nasa['chord'][-13]  # Assuming the first entry corresponds to the tip chord
    print(f"Using tip chord length from NASA data for blade design: {C_TIP:.4f} m")

    
    drones = [dualCopter, quadCopter]
    
    print("initialize best dual")
    dual_blade_design = BladeDesign(drone=dualCopter, planet=mars)
    print("initialize best quad")
    quad_blade_design = BladeDesign(drone=quadCopter, planet=mars)
    
    blade_designs = [dual_blade_design, quad_blade_design]
    computed_blade_designs = []

    print("Computing blade designs...")
    
    for blade_design in blade_designs:
        blade_design.compute_no_twist()
        print(f"Computed no twist for {blade_design.__class__.__name__}")
        blade_design.compute_linear_twist()
        print(f"Computed linear twist for {blade_design.__class__.__name__}")
        blade_design.compute_optimum_twist()
        print(f"Computed optimum twist for {blade_design.__class__.__name__}")
        blade_design.compute_optimum_plan_form_and_twist(C_TIP)
        print(f"Computed optimum plan form and twist for {blade_design.__class__.__name__}")
        computed_blade_designs.append(blade_design)
        print(f"Appended {blade_design.__class__.__name__} to computed_blade_designs")

    blade_designs = computed_blade_designs        
        
    plot_q5_twist_subplots(blade_designs=blade_designs, blade_geometry_nasa=blade_geometry_nasa,
        filename="plots/q5_twist_distributions.png"
    )
    
    plot_q5_chord_distribution(blade_designs=blade_designs, blade_geometry_nasa=blade_geometry_nasa,
        filename="plots/q5_chord_distribution.png")
    
    #TODO IMPLEMENT BEM
    
    # dual_blade_design.bem()
    # dual_blade_design.total_thrust = np.sum(dual_blade_design.dT_be)
    # dual_blade_design.compute_total_power()  
    
    #TODO Plot the distribution of 𝑑𝐶𝑇 (𝑑𝐶𝑇 = 𝑑𝑇/(𝜌𝐴𝑉𝑡𝑖𝑝2 )) and 𝑑𝑃 vs the non-dimensional radius. 
    
    #TODO For your final design, calculate the total power required by the rotor and thrust delivered, and compare it with your result from Task 2. Your final design must be capable of delivering the desired thrust. 
    print("\ntwist and chord distribution test computations DONE\n")

if do["bem_template"]:
    print("\n################### BEM TEMPLATE ###################\n")
    drone = DroneDesign(
                reference=ingenuity,
                name="bemDrone",
                rotor_diameter=0.76,
                chord=0.055,
                N_blades=2,
                N_rotors=2,
                rpm=2800,
                N_batteries=48,
                payload_mass=0.0,
                aux_components_mass=1.0
            )
    
    drone.solve_mass_power(P_initial=ingenuity.hover_power, planet=mars)
    # load reference data from ingenuity nasa report for twist distribution and chord distribution    
    blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
    NO_BLADE_ELEMENTS = len(blade_geometry_nasa['y'])
    # print(f"Number of blade elements based on NASA data: {NO_BLADE_ELEMENTS}")
    
    # TEST BEM ON INGENUITY
    ref_blade = BladeDesign(drone=ingenuity, planet=mars, c_tip=C_MEAN, no_blade_elements=NO_BLADE_ELEMENTS)
    ref_blade.set_chord(blade_geometry_nasa['chord'])
    ref_blade.set_twist(blade_geometry_nasa['twist_deg'])
    # print("We assume airfoil clf 5605 across the whole blade, even though its a simplification, since we cannot get cl/cd experimental for station 1,2,3,4 and xfoil fails at these low Reynolds numbers. We are afterall only using the last 80\% of the blade, and clf5605 is used from 40\% to 80\% anyways so its only 20\% of the rotor that has a different airfoil that we are simplifying compared to our own blade design.\n")
    print("\INGENUITY NON-LINEAR W. DIMENSIONS\n")
    ingenuity.bem_master(ref_blade, linear=False, dimensionless=False)
    ingenuity.print_bem_results()
    # print(f"Induced velocity for reference blade: {ref_blade.v_i}")
    
    
    # We can use the tip chord length from the reference data to more accurately compute the chord distribution
    C_TIP = blade_geometry_nasa['chord'][-13]  # Assuming the first entry corresponds to the tip chord
    # print(f"Using tip chord length from NASA data for blade design: {C_TIP:.4f} m")

    blade = BladeDesign(drone=drone, planet=mars, c_tip=C_TIP, no_blade_elements=NO_BLADE_ELEMENTS)
    blade.compute_optimum_plan_form_and_twist()
    blade.set_chord(blade.optimum_chord_distribution)
    blade.set_twist(blade.theta_optimum_plan_form)
    
    print("\DESIGN BLADE NON-LINEAR W. DIMENSIONS\n")
    drone.bem_master(blade, linear=False, dimensionless=False)
    drone.print_bem_results()
    
if do["Q5"]:
    print("\n################### Q5 BEM CHORD OPTIMIZATION ###################\n")
    print("\nRunning Q5 BEM and designing blades based on optimum design\n")

    # load reference data from ingenuity nasa report for twist distribution and chord distribution    
    blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
    NO_BLADE_ELEMENTS = len(blade_geometry_nasa['y'])

    # so we found out with "bem" and "bem_master" that our chord is likely too small.
    # we also found out that our bem likely has some bugs, since we cant even produce enough torque for ingenuity
    # anyways, lets try to iterate through chord lengths and see if we can get enough torque for our design, with the twist we have already computed
    
    # We can use the tip chord length from the reference data as the shortest chord
    C_TIP = blade_geometry_nasa['chord'][-13]  # Assuming the first entry corresponds to the tip chord

    n_iter = 5
    c_tip_array = np.linspace(C_TIP, 0.1, 100)
    # define desired thrust and power margins
    thrust_margin = 0.1  # 10% more thrust than required
    power_margin = 0.1   # 10% more power than required
    
    # print all current stats about our dualCopter before we start the sweep
    print("\n############### DUALCOPTER STATS ###############\n")
    print_drone_stats(dualCopter)
    
    print("\n############### QUADCOPTER STATS ###############\n")
    print_drone_stats(quadCopter)

    
    
    
    def q5_bem_chord_sweep(drone: Drone, c_tip_array: np.ndarray, planet: Planet, thrust_margin: float, power_margin: float):
        blade_designs = []
        drone_designs = []
        
        # iterate through array of tip chord lengths
        for c_tip in c_tip_array:
            # (1)   because we want to use the full chord distribution now, we need to generate a drone instance, so we can generate a blade instance
            #       with starting point in this example drone instance, we can iterate over a c_tip array, to first generate a chord distributtion, and then generate a drone based on that
            drone = DroneDesign(
                        reference=ingenuity,
                        name=drone.name,
                        rotor_diameter=drone.rotor_diameter,
                        chord=c_tip,
                        N_blades=drone.N_blades,
                        N_rotors=drone.N_rotors,
                        rpm=drone.rpm,
                        N_batteries=drone.N_batteries,
                        payload_mass=drone.payload_mass,
                        aux_components_mass=drone.aux_components_mass
                    )
            drone.solve_mass_power(P_initial=ingenuity.hover_power, planet=planet)

            # (2)   generate blade instance based on the drone, to get twist and chord
            blade = BladeDesign(drone=drone, planet=planet, c_tip=c_tip, no_blade_elements=NO_BLADE_ELEMENTS)
            blade.compute_optimum_plan_form_and_twist()
            blade.set_chord(blade.optimum_chord_distribution)
            
            # (3)    generate a new drone with this chord distribution
            drone = DroneDesign(
                        reference=ingenuity,
                        name=drone.name,
                        rotor_diameter=drone.rotor_diameter,
                        chord=blade.chord,
                        N_blades=drone.N_blades,
                        N_rotors=drone.N_rotors,
                        rpm=drone.rpm,
                        N_batteries=drone.N_batteries,
                        payload_mass=drone.payload_mass,
                        aux_components_mass=drone.aux_components_mass
                    )
            
            drone.solve_mass_power(P_initial=ingenuity.hover_power, planet=planet)
            
            # (4) finally, with the drone based on the new chord and twist, we can generate the blade again since the calculations are based on parameters from the drone
            blade = BladeDesign(drone=drone, planet=planet, c_tip=c_tip, no_blade_elements=NO_BLADE_ELEMENTS)
            blade.compute_optimum_plan_form_and_twist()
            blade.set_chord(blade.optimum_chord_distribution)
            blade.set_twist(blade.theta_optimum_plan_form)
            
            # (5) Finally we can run BEM specific to this drone and blade design
            drone.bem_master(blade, linear=False, dimensionless=False)
            blade_designs.append(blade)
            drone_designs.append(drone)
        
        # get design with c_tip that meets thrust requirements    
        optimal_drone = None
        optimal_blade = None
        optimal_c_tip  = None

        for c_tip, drone, blade in zip(c_tip_array, drone_designs, blade_designs):
            thrust_ok = drone.total_thrust_generation >= drone.total_thrust * (1 + thrust_margin)
            
            if thrust_ok:
                optimal_drone = drone
                optimal_blade = blade
                optimal_c_tip  = c_tip
                break  # smallest c_tip meeting thrust requirement

        if optimal_drone is not None:
            print(f"\nOptimal tip chord: {optimal_c_tip:.4f} m")
            optimal_drone.print_bem_results()
        else:
            print("No design meets thrust requirement. Increase c_tip range.")
        
        
        # before returning, lets compute the twist distribution with the other methods as well
        twist_distributions = []

        if optimal_blade and optimal_drone is not None:
            print("Computing twist distributions with different methods...")
            optimal_blade.compute_no_twist()
            optimal_blade.compute_linear_twist()
            optimal_blade.compute_optimum_twist()
            optimal_blade.compute_optimum_plan_form_and_twist()
            
            # return the optimal drone, blade, and the list of all designs for plotting
            return optimal_drone, optimal_blade, drone_designs, blade_designs
    
        else:
            print("No design meets thrust requirement for quadcopter. Increase c_tip range or look into BEM implementation.")
            return None, None, drone_designs, blade_designs
    
    
    dualCopter, dual_blade_design, dual_designs, dual_blade_designs = q5_bem_chord_sweep(
            drone=dualCopter,
            c_tip_array=c_tip_array,
            planet=mars,
            thrust_margin=thrust_margin,
            power_margin=power_margin)
        
    quadCopter, quad_blade_design, quad_designs, quad_blade_designs = q5_bem_chord_sweep(
            drone=quadCopter,
            c_tip_array=c_tip_array,
            planet=mars,
            thrust_margin=thrust_margin,
            power_margin=power_margin)

    # now we want to see which designs produce enough torque and power
    # therefore, lets make this plot:
    # x axis: c_tip
    # subplot 1: torque
    # subplot 2: power
    # horizontal lines: required torque and power for hover
    # hint: values stored like this:
    # print(f"Total power generated by BEM: {self.total_power_generation:.2f} W, Total thrust generated by BEM: {self.total_thrust_generation:.2f} N")
    # print(f"Required hover power from mass-power solver: {self.hover_power:.2f} W, required thrust: {self.total_thrust:.2f} N\n")
    
    # plot only the dual
    plot_q5_bem_chord_sweep(dual_designs,
                            c_tip_array,
                            thrust_margin=thrust_margin,
                            power_margin=power_margin,
                            filename="plots/q5_bem_chord_sweep.png")

    # plot both
    plot_q5_bem_chord_sweep_double(dual_designs, quad_designs,
                                   c_tip_array,
                                   thrust_margin,
                                   power_margin,
                                   filename="plots/q5_bem_chord_sweep_double.png",
                                   )
    # write the chord distribution of the optimal blade to a text file, so we can use it for the final design in the report
    # print(dualCopter.chord)
    
    
    #TODO Plot the distribution of 𝑑𝐶𝑇 (𝑑𝐶𝑇 = 𝑑𝑇/(𝜌𝐴𝑉𝑡𝑖𝑝2 )) and 𝑑𝑃 vs the non-dimensional radius. 
    # dCT stored as: drone.dCT
    # dP stored as: drone.dPower
    plot_q5_dCT_dP_distribution(
        dual_blade=dual_blade_design,
        quad_blade=quad_blade_design,
        filename="plots/q5_dCT_dP_distribution.png"
    )

    #TODO For your final design, calculate the total power required by the rotor and thrust delivered, and compare it with your result from Task 2. Your final design must be capable of delivering the desired thrust. 

    
    print("\nQ5 BEM CHORD OPTIMIZATION DONE\n")
    
if do["bem"]:
    print("\n ############ BEM THREE SEPERATE FUNCTIONS TEST ############\n")
    drone = DroneDesign(
                reference=ingenuity,
                name="bemDrone",
                rotor_diameter=0.76,
                chord=0.055,
                N_blades=2,
                N_rotors=2,
                rpm=2800,
                N_batteries=48,
                payload_mass=0.0,
                aux_components_mass=1.0
            )
    
    drone.solve_mass_power(P_initial=ingenuity.hover_power, planet=mars)
    # load reference data from ingenuity nasa report for twist distribution and chord distribution    
    blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
    NO_BLADE_ELEMENTS = len(blade_geometry_nasa['y'])
    # print(f"Number of blade elements based on NASA data: {NO_BLADE_ELEMENTS}")
    
    # TEST BEM ON INGENUITY
    # print("\nTesting BEM on ingenuity reference design\n")
    ref_blade = BladeDesign(drone=ingenuity, planet=mars, c_tip=C_MEAN, no_blade_elements=NO_BLADE_ELEMENTS)
    ref_blade.set_chord(blade_geometry_nasa['chord'])
    ref_blade.set_twist(blade_geometry_nasa['twist_deg'])
    # print("We assume airfoil clf 5605 across the whole blade, even though its a simplification, since we cannot get cl/cd experimental for station 1,2,3,4 and xfoil fails at these low Reynolds numbers. We are afterall only using the last 80\% of the blade, and clf5605 is used from 40\% to 80\% anyways so its only 20\% of the rotor that has a different airfoil that we are simplifying compared to our own blade design.\n")
    print("\INGENUITY NON-LINEAR W. DIMENSIONS\n")
    ingenuity.bem(ref_blade)
    ingenuity.print_bem_results()
    # print(f"Induced velocity for reference blade: {ref_blade.v_i}")
    
    print("\INGENUITY LINEAR NON-DIMENSIONAL\n")
    ingenuity.bem_linear(ref_blade)
    ingenuity.print_bem_results()
    
    # We can use the tip chord length from the reference data to more accurately compute the chord distribution
    C_TIP = blade_geometry_nasa['chord'][-13]  # Assuming the first entry corresponds to the tip chord
    # print(f"Using tip chord length from NASA data for blade design: {C_TIP:.4f} m")

    # print("computing twist and chord distribution for bemDrone (full bem)")
    blade = BladeDesign(drone=drone, planet=mars, c_tip=C_TIP, no_blade_elements=NO_BLADE_ELEMENTS)
    blade.compute_optimum_plan_form_and_twist()
    blade.set_chord(blade.optimum_chord_distribution)
    blade.set_twist(blade.theta_optimum_plan_form)
    print("\DESIGN BLADE NON-LINEAR W. DIMENSIONS\n")
    drone.bem(blade)
    drone.print_bem_results()
    # print(f"Induced velocity for design blade: {blade.v_i}")
    
    # print("computing twist and chord distribution for bemDrone (dimensionless)\n")
    print("\DESIGN BLADE NON-LINEAR DIMENSIONLESS\n")
    drone.bem_dimensionless(blade)
    drone.print_bem_results()
    # print(f"Induced velocity for design blade with dimensionless approach: {blade.v_i}")
    
    print("\DESIGN BLADE LINEAR NON-DIMENSIONAL\n")
    drone.bem_linear(blade)
    drone.print_bem_results()
    # print(f"Induced velocity for design blade with linear approach: {blade.v_i}")

if do["bem_master"]:
    print("\n################### BEM MASTER ###################\n")
    drone = DroneDesign(
                reference=ingenuity,
                name="bemDrone",
                rotor_diameter=0.76,
                chord=0.055,
                N_blades=2,
                N_rotors=2,
                rpm=2800,
                N_batteries=48,
                payload_mass=0.0,
                aux_components_mass=1.0
            )
    
    drone.solve_mass_power(P_initial=ingenuity.hover_power, planet=mars)
    # load reference data from ingenuity nasa report for twist distribution and chord distribution    
    blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
    NO_BLADE_ELEMENTS = len(blade_geometry_nasa['y'])
    # print(f"Number of blade elements based on NASA data: {NO_BLADE_ELEMENTS}")
    
    # TEST BEM ON INGENUITY
    ref_blade = BladeDesign(drone=ingenuity, planet=mars, c_tip=C_MEAN, no_blade_elements=NO_BLADE_ELEMENTS)
    ref_blade.set_chord(blade_geometry_nasa['chord'])
    ref_blade.set_twist(blade_geometry_nasa['twist_deg'])
    # print("We assume airfoil clf 5605 across the whole blade, even though its a simplification, since we cannot get cl/cd experimental for station 1,2,3,4 and xfoil fails at these low Reynolds numbers. We are afterall only using the last 80\% of the blade, and clf5605 is used from 40\% to 80\% anyways so its only 20\% of the rotor that has a different airfoil that we are simplifying compared to our own blade design.\n")
    print("\INGENUITY NON-LINEAR W. DIMENSIONS\n")
    ingenuity.bem_master(ref_blade, linear=False, dimensionless=False)
    ingenuity.print_bem_results()
    # print(f"Induced velocity for reference blade: {ref_blade.v_i}")
    
    print("\INGENUITY LINEAR NON-DIMENSIONAL\n")
    ingenuity.bem_master(ref_blade, linear=True, dimensionless=True)
    ingenuity.print_bem_results()
    
    print("\INGENUITY NON-LINEAR DIMENSIONLESS\n")
    ingenuity.bem_master(ref_blade, linear=False, dimensionless=True)
    ingenuity.print_bem_results()
    
    # We can use the tip chord length from the reference data to more accurately compute the chord distribution
    C_TIP = blade_geometry_nasa['chord'][-13]  # Assuming the first entry corresponds to the tip chord
    # print(f"Using tip chord length from NASA data for blade design: {C_TIP:.4f} m")

    print("computing twist and chord distribution for bemDrone (full bem)")
    blade = BladeDesign(drone=drone, planet=mars, c_tip=C_TIP, no_blade_elements=NO_BLADE_ELEMENTS)
    blade.compute_optimum_plan_form_and_twist()
    blade.set_chord(blade.optimum_chord_distribution)
    blade.set_twist(blade.theta_optimum_plan_form)
    
    print("\DESIGN BLADE NON-LINEAR W. DIMENSIONS\n")
    drone.bem_master(blade, linear=False, dimensionless=False)
    drone.print_bem_results()
    # print(f"Induced velocity for design blade: {blade.v_i}")
    
    print("\DESIGN BLADE LINEAR NON-DIMENSIONAL\n")
    drone.bem_master(blade, linear=True, dimensionless=True)
    drone.print_bem_results()
    # print(f"Induced velocity for design blade with dimensionless approach: {blade.v_i}")
    
    print("\DESIGN BLADE NON-LINEAR DIMENSIONLESS\n")
    drone.bem_master(blade, linear=False, dimensionless=True)
    drone.print_bem_results()
    # print(f"Induced velocity for design blade with linear approach: {blade.v_i}")


if do["Q6"]:
    print("\nRunning Q6 and performing final analysis\n")
    
    #TODO Forward flight and stuff like that
    print("\nQ6 DONE\n")
    
    
if do["MASTER_LOOP"]:
    print("\n################### MASTER LOOP ###################\n")
    print("\nRunning master loop to execute all tasks in order because chord, radius and blades are all intertwined\n")
    
    # Q1 MUST BE RUN FIRST
    
    # FOR THE FINAL STEPS WE ONLY USE DUALCOPTER
    
    
    
    n_iter = 50
    # N_blades_array = np.array([2, 3, 4, 5])
    N_blades_array = np.array([2])
    min_radius = 0.15
    max_radius = 1.5
    diameter_array = np.linspace(min_radius*2, max_radius*2, n_iter)
    
    initial_chord = ingenuity.chord
    
    iteration_count = 0
    max_iterations = 20
    tol = 0.01  # 1% tolerance for convergence
    diff = np.inf
    
    best_designs = []
    
    while diff > tol and iteration_count < max_iterations:
        if iteration_count == 0:
            chord = initial_chord 
        
        print(f"Iteration {iteration_count}")
        iteration_count += 1

        print("grid search q2 radius and blades")
        # iterate through diameters (and blades)                
        dualCopter, quad_candidate, converged_drone_designs = grid_search_q2(
            diameters=diameter_array,
            N_blades=N_blades_array,
            chord=chord,
            planet=mars,
            reference_drone=ingenuity
        )

        # keep going even if quad fails
        quad_for_q3 = quad_candidate if quad_candidate is not None else dualCopter
        print("\noptimal radius and blades from grid search:", dualCopter.rotor_radius, dualCopter.N_blades)
        
        print("battery sweep q3 with optimal radius and blades from q2")
        # iterate through battery count
        dualCopter, _, results, N_batt_max_2kg = battery_sweep_q3(
            dualCopter=dualCopter,
            quadCopter=quad_for_q3,
            planet=mars,
            chord=chord
        )

        
        n_iter = 50
        # this part is confusing but hold on:
        # in the previous steps, we hold the chord distribution constant to find optimal radius, nblades and batteries
        # now, we want to iterate through chord distributions based on a tip chord length
        # that tip chord will be based on the mean of the chord distribution we have been using so far
        C_TIP = np.mean(chord)
        c_tip_array = np.linspace(C_TIP, 4*C_TIP, 100)
        # define desired thrust and power margins
        thrust_margin = 0.1  # 10% more thrust than required
        power_margin = 0.1   # 10% more power than required
        print("chord sweep q5 with optimal radius and blades from q3")
        # iterate through chord lengths
        dualCopter, dual_blade_design, dual_designs, dual_blade_designs = q5_bem_chord_sweep(
                drone=dualCopter,
                c_tip_array=c_tip_array,
                planet=mars,
                thrust_margin=thrust_margin,
                power_margin=power_margin)
        
        best_designs.append(dualCopter)
        # check if the chord distribution changed significantly
        diff = float(np.max(np.abs(dualCopter.chord - chord)))
              
        chord = dualCopter.chord
    
    # plot the final design
    # 2x2 plot
    # subplot1: flight time vs rotor radius
    # subplot2: hover power vs rotor radius
    # subplot3: thrust vs tip chord length
    # subplot4: power vs tip chord length
    plot_q5_master_loop_summary(
        best_designs=best_designs,
        dual_designs=dual_designs,
        c_tip_array=c_tip_array,
        filename="plots/master_loop_summary.png",
    )


