import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from tqdm import tqdm
import numpy as np
from dataclasses import dataclass, field
import pickle
from planet import Planet
from drone import Drone, Ingenuity, DroneDesign, Aircraft
from blade_design import BladeDesign, WingDesign
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
                        plot_q5_master_loop_summary,
                        plot_master_loop_2x2,
                        plot_master_loop_3x2,
                        plot_q5_twist,
                        plot_q5_dCT_dP_distribution,
                        plot_q5_dCT_dP_distribution_comparison,
                        plot_q6_wingspan_sweep,
                        plot_q6_forward_flight
                        )
from print_funcs import (print_q3_optimums,
                         print_q2_comparison_latex,
                         print_q4_reynolds_latex,
                         print_drone_stats
                        )
from parse_txt_funcs import(read_polar_txt,
                           read_blade_geometry_nasa,
                           load_rotor_parameters,
                            xfoil_polar_txt_to_dataframe
                           )

@dataclass
class DesignResult:
    radius: float
    c_mean: float
    drone: object = field(repr=False)
    N_rotors: int = 0
    N_batteries: int = 0
    flight_time: float = 0.0
    total_thrust_generation: float = 0.0
    total_power_generation: float = 0.0
    total_thrust_required: float = 0.0
    total_power_required: float = 0.0
    aspect_ratio: float = 0.0
    
 

#TODO DO THE FINAL GRID SEARCH

do = {
    "Q1": True,
    "Q2": True,
    "Q3": True,
    "Q4": True,
    "Q5_MASTER_LOOP": True,
    "Q5_analysis": True,
    "Q5_twist_chord_dCt_dP": True,
    "Q5_twist_chord_dCt_dP_comparison": True,
    "Q6": True,
    "Q6_2d_grid_search": True,   
    }

if do["Q1"]:
    print("\n################### Q1 ###################\n")
    print("\nRunning Q1 and generating performance metrics for Ingenuity reference design on planet: Mars\n")
    
    mars = Planet("Mars", g=3.72, rho=0.01503, viscosity=1.3e-5)
    
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
    
    # define number of blade elements for the blade design
    NO_BLADE_ELEMENTS = len(blade_geometry_nasa["y"])
    
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
    
    MAX_EXTRA_PAYLOAD = 2.0  # kg
    MASS_PER_BATTERY = float(ingenuity.mass_per_battery)
    BASE_BATTERIES = int(ingenuity.N_batteries)

    # Max extra batteries allowed by 2 kg payload constraint
    N_EXTRA_MAX_2KG = int(np.floor(MAX_EXTRA_PAYLOAD / MASS_PER_BATTERY))
    N_BATT_MAX_2KG = BASE_BATTERIES + N_EXTRA_MAX_2KG
    print("\nQ1 DONE \n")

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
                    result = drone.solve_mass_power(P_initial, planet)
                    # result = drone.compute_planet_performance(planet)

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
    # chord_opt_q5 = np.array([1.37878788, 1.21350071, 1.08360023, 0.97882126, 0.89251894, 0.82020202,
    #                          0.75872578, 0.70582257, 0.65981598, 0.61943995, 0.58372042, 0.55189577,
    #                          0.52336189, 0.49763346, 0.47431611, 0.4530861,  0.43367515, 0.41585906,
    #                          0.39944904, 0.38428494, 0.37023008, 0.35716703, 0.34499438, 0.3336241,
    #                          0.32297939, 0.31299294, 0.30360553, 0.29476482, 0.28642442, 0.27854301])
    
    chord_opt_q5 = np.array(C_MEAN, dtype=float) * np.ones(NO_BLADE_ELEMENTS)  # using the mean chord from the NASA data for the chord distribution in Q2 grid search
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

if do["Q5_MASTER_LOOP"]:
    print("\n################### MASTER LOOP ###################\n")
    print("\nRunning master loop to execute all tasks in order because chord, radius and blades are all intertwined\n")
    
    
    # Q1 MUST BE RUN FIRST
    
    # FOR THE FINAL STEPS WE ONLY USE QUADCOPTER
    


    print(f"\nMass per battery: {MASS_PER_BATTERY:.4f} kg")
    print(f"Base batteries: {BASE_BATTERIES} batteries")
    print(f"Max extra batteries within 2 kg: {N_EXTRA_MAX_2KG}")
    print(f"Max total batteries within 2 kg: {N_BATT_MAX_2KG}")

        
    def master_loop(n_rotors,
                    n_radius: int,
                    n_chords: int,
                    chord_distribution: np.ndarray,
                    BASE_BATTERIES: int=BASE_BATTERIES,
                    N_EXTRA_MAX_2KG: int=N_EXTRA_MAX_2KG,
                    ingenuity_chord: bool=False,
                    planet: Planet=mars,
                    reference_drone: Drone=ingenuity,
                    N_blades: int=2,
                    rpm: int=2800,
                    payload_mass: float=0.0,
                    aux_components_mass: float=1.0
                    ):
        
        # load ingenuity blade geometry
        blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
        
        # define the range of rotor radii and chord lengths to sweep through
        radius_array = np.linspace(reference_drone.rotor_radius/4, reference_drone.rotor_radius*2, n_radius)
        chord_array = np.linspace(chord_distribution/5, chord_distribution, n_chords)

        # Evaluate beyond the 2 kg limit so unconstrained optimum can be found
        N_batt_eval = np.arange(BASE_BATTERIES, BASE_BATTERIES + 2 * N_EXTRA_MAX_2KG + 1, dtype=int)
        
        # initialize dicts to store designs
        designs: list[DesignResult] = []
        
        tot_iterations = n_radius * n_chords * len(N_batt_eval)
        iteration_count = 0
        
        pbar = tqdm(
            total=tot_iterations,
            desc="Master loop",
            unit="it",
            ncols=100
        ) if tqdm is not None else None

        for radius in radius_array:
            for chord_dist in chord_array:
                for N_batteries in N_batt_eval:
                    pbar.update(1)
                    # define mean chord for naming and c_tip blade design
                    c_mean = np.mean(chord_dist)

                    iteration_count += 1
                    # print(f"Master loop progress: {iteration_count/tot_iterations*100:.0f}%")
                    drone = DroneDesign(
                        reference=reference_drone,
                        name=f"QuadCopter_R{radius:.2f}_C{c_mean:.3f}",
                        rotor_diameter=radius*2,
                        chord=chord_dist,
                        N_blades=N_blades,
                        N_rotors=n_rotors,
                        rpm=rpm,
                        N_batteries=N_batteries,
                        payload_mass=payload_mass,
                        aux_components_mass=aux_components_mass
                    )
                    
                    drone.solve_mass_power(P_initial=reference_drone.hover_power, planet=planet)
                    drone.compute_planet_performance(planet)
                    if drone is not None and drone.total_thrust < 1e10:
                        # generate blade design based on the drone design if the mass power solver converges
                        blade = BladeDesign(drone=drone, planet=planet, c_tip=c_mean, no_blade_elements=NO_BLADE_ELEMENTS)
                        # print(drone.rotor_mass)
                            
                        # since we already have a chord distribution, we only compute the twist here. 
                        # (the chord distribtuion that would be generated, would just be the same distribution scaled,
                        # so by looping over radii and chord distributions scaled, we are alreadyd doing this))
                        
                        
                        # if we use the chord distribution from our blade design, we compute the twist based on that chord distribution.
                        if ingenuity_chord == False:
                            blade.compute_optimum_plan_form_and_twist()
                            blade.set_twist(blade.theta_optimum_plan_form)
                            blade.set_chord(blade.optimum_chord_distribution)
                        # if we use the ingenuity chord distribution, we already have twist. 
                        else:
                            blade.set_twist(blade_geometry_nasa['twist_deg'])
                            blade.set_chord(chord_dist)
                        
                        # compute bem for the blade and sum it up for n rotors and n blades specific for this drone design
                        drone.bem(blade, linear=False, dimensionless=False)
                        
                        # if drone.total_thrust_generation >= drone.total_thrust * thrust_margin and drone.total_power_generation >= drone.hover_power * power_margin:
                        
                        # Store the design in the dict
                        designs.append(DesignResult(         
                            radius=radius,
                            c_mean=c_mean,
                            drone=drone,
                            N_rotors=n_rotors,
                            N_batteries=N_batteries,
                            flight_time=drone.total_hover_time,
                            total_thrust_generation=drone.total_thrust_generation,
                            total_power_generation=drone.total_power_generation,
                            total_thrust_required=drone.total_thrust,
                            total_power_required=drone.hover_power,
                            aspect_ratio=drone.aspect_ratio
                        ))
        
        pbar.close()
        print(f"\nMaster loop done after {iteration_count}.\n")
        return designs

    # run the master loop and save the designs to a dataclass object
    designs_ingenuity_chord = master_loop(
        n_rotors=4,
        n_radius=40,
        n_chords=40,
        chord_distribution=ingenuity.chord,
        ingenuity_chord=True,  # use the chord distribution from ingenuity for the master loop
        )
    
    # generate a blade based on ingenuity, to get the smallest optimal chord distribution that we can scale with the master loop
    blade_optimal_chord = BladeDesign(
        drone=ingenuity,
        planet=mars,
        c_tip=np.mean(ingenuity.chord))
    blade_optimal_chord.compute_optimum_plan_form_and_twist()
    blade_optimal_chord.set_chord(blade_optimal_chord.optimum_chord_distribution)
    
    designs_optimal_chord = master_loop(
        n_rotors=4,
        n_radius=40,
        n_chords=40,
        chord_distribution=blade_optimal_chord.chord,
        ingenuity_chord=False,  # use the chord distribution from ingenuity for the master loop
        )

    
    # save the designs to a pickle file, so we can load it later for analysis without having to run the master loop again
    with open("results/master_loop_designs_ingenuity_chord.pkl", "wb") as f:
        pickle.dump(designs_ingenuity_chord, f)
        pickle.dump(designs_optimal_chord, f)
        
    print("Master loop designs saved to results")

if do["Q5_analysis"]:
    print("\n################### MASTER LOOP ANALYSIS ###################\n")
    with open("results/master_loop_designs_ingenuity_chord.pkl", "rb") as f:
        designs_ingenuity_chord = pickle.load(f)
        designs_optimal_chord = pickle.load(f)
    print(f"Loaded {len(designs_ingenuity_chord)} designs from master loop.")
    
    # lets do some plots to understand the results better
    """   radius: float
        c_mean: float
        drone: object=field(repr=False)
        N_rotors: int
        total_thrust_generation: float
        total_power_generation: float
        total_thrust_required: float
        total_power_required: float
        AR: float
    """
    # 2x2 subplot with:
    # left x axis: radius
    # right x axis: c_mean
    # subplot 1: total thrust generation and total thrust required
    # subplot 2: total power generation and total power required
    # AR: invidivudal for each drone design, so lets use that as the legend name, and give a specific color to each AR
    

    from itertools import groupby

    def select_optimal_battery(designs: list[DesignResult], max_batteries: int) -> list[DesignResult]:
        """For each (radius, c_mean) pair, keep the design with max hover time within the battery constraint."""
        groups = {}
        for d in designs:
            if d.N_batteries > max_batteries:
                continue
            key = (round(d.radius, 6), round(d.c_mean, 6))
            if key not in groups or d.flight_time > groups[key].flight_time:
                groups[key] = d
        return list(groups.values())
    
    designs_ingenuity_chord = select_optimal_battery(designs_ingenuity_chord, max_batteries=N_BATT_MAX_2KG)
    designs_optimal_chord = select_optimal_battery(designs_optimal_chord, max_batteries=N_BATT_MAX_2KG)


    # plot_master_loop_2x2(designs_ingenuity_chord, filename="plots/master_loop_analysis_2x2_ingenuity_chord.png")
    # plot_master_loop_2x2(designs_optimal_chord, filename="plots/master_loop_analysis_2x2_optimal_chord.png")

    plot_master_loop_3x2(designs_ingenuity_chord, main_title="Ingenuity Chord", filename="plots/master_loop_analysis_3x2_ingenuity_chord.png")
    plot_master_loop_3x2(designs_optimal_chord, main_title="Optimal Chord", filename="plots/master_loop_analysis_3x2_optimal_chord.png")

    # extract the best design based on hover time for each master loop
    def extract_best_design(designs: list[DesignResult], thrust_margin: float, power_margin: float) -> Drone | None:
        valid = [
            d for d in designs
            if d.total_thrust_generation >= d.total_thrust_required * thrust_margin
            and d.total_power_generation  >= d.total_power_required  * power_margin
        ]
        if not valid:
            print("No valid designs found meeting thrust and power margins.")
            return None
        return max(valid, key=lambda d: d.flight_time).drone

    best_ingenuity = extract_best_design(designs_ingenuity_chord, thrust_margin=1.1, power_margin=1.1)
    best_optimal   = extract_best_design(designs_optimal_chord,   thrust_margin=1.1, power_margin=1.1)
    
    # print stats about the best designs
    print("\nBest design from master loop with ingenuity chord distribution:\n")
    best_ingenuity.print_stats()
    print(best_ingenuity.to_latex_table("Ingenuity Chord Distribution"))
    print("\nBest design from master loop with optimal chord distribution:\n")
    best_optimal.print_stats()
    print(best_optimal.to_latex_table("Optimal Chord Distribution"))
    
    # save best design to pickle file
    with open("results/best_designs.pkl", "wb") as f:
        pickle.dump(best_ingenuity, f)
        pickle.dump(best_optimal, f)

if do["Q5_twist_chord_dCt_dP"]:
    print("\n################### TWIST, CHORD, dCT and dP DISTRIBUTIONS ###################\n")
    # load the best design from the master loop
    with open("results/best_designs.pkl", "rb") as f:
        best_ingenuity = pickle.load(f)
        best_optimal   = pickle.load(f)
    
    # generate the twist distribtuions
    quadCopter = best_optimal
    blade = BladeDesign(drone=quadCopter, planet=mars, c_tip=np.mean(quadCopter.chord))
    blade.compute_no_twist()
    blade.compute_linear_twist()
    blade.compute_optimum_twist()
    blade.compute_optimum_plan_form_and_twist()
    blade.set_twist(blade.theta_optimum_plan_form)
    blade.set_chord(blade.optimum_chord_distribution)
    blade.bem()
    plot_q5_twist(blade, blade_geometry_nasa=blade_geometry_nasa, filename="plots/q5_twist_distributions.png")
    plot_q5_chord_distribution(blade_design=blade, blade_geometry_nasa=blade_geometry_nasa, filename="plots/q5_chord_distribution.png")
    plot_q5_dCT_dP_distribution(blade, filename="plots/q5_dCT_dP_distribution_best_design.png")

if do["Q5_twist_chord_dCt_dP_comparison"]:
    import copy
    print("\n################### TWIST, CHORD, dCT and dP DISTRIBUTIONS COMPARISON ###################\n")

    with open("results/best_designs.pkl", "rb") as f:
        best_ingenuity = pickle.load(f)
        best_optimal   = pickle.load(f)

    quadCopter = best_optimal
    

    def compare_twists(constant_chord: bool=False):
        
        # compute all twist distributions from a single reference blade
        ref_blade = BladeDesign(drone=quadCopter, planet=mars, c_tip=np.mean(quadCopter.chord), no_blade_elements=NO_BLADE_ELEMENTS)
        ref_blade.compute_no_twist()
        ref_blade.compute_linear_twist()
        ref_blade.compute_optimum_twist()
        ref_blade.compute_optimum_plan_form_and_twist()
        
        chord_distribution = ref_blade.optimum_chord_distribution
        if constant_chord:
            chord_distribution = np.full_like(chord_distribution, np.mean(chord_distribution))        

        twist_cases = [
            ("No Twist",          ref_blade.no_twist),
            ("Linear Twist",      ref_blade.linear_twist),
            ("Optimum Twist",     ref_blade.optimum_twist),
            ("Optimum Plan Form", ref_blade.theta_optimum_plan_form),
        ]

        bems       = []   # blade objects after BEM (for dCT/dP plots)
        drone_bems = []   # independent drone copies after BEM (for thrust/power)

        for label, twist in twist_cases:
            # fresh blade with the twist for this case
            b = BladeDesign(drone=quadCopter, planet=mars, c_tip=np.mean(quadCopter.chord), no_blade_elements=NO_BLADE_ELEMENTS)
            b.set_chord(chord_distribution)
            b.set_twist(twist)
            b.bem(linear=False, dimensionless=False)
            bems.append(b)

            # deep-copy drone so each case has its own independent thrust/power state
            drone_copy = copy.deepcopy(quadCopter)
            drone_copy.bem(b, linear=False, dimensionless=False)
            drone_bems.append(drone_copy)

        labels = [label for label, _ in twist_cases]

        plot_q5_dCT_dP_distribution_comparison(
            bems,
            labels=labels,
            title="Quad Copter",
            filename=f"plots/q5_dCT_dP_distribution_comparison_constant_chord_{constant_chord}.png"
        )

        # latex table of thrust and power per twist distribution
        def print_latex_table(drone_bems, labels):
            lines = [
                r"\begin{table}[H]",
                r"\centering",
                r"\begin{tabular}{lcc}",
                r"\hline",
                r"Twist Distribution & Thrust Generated (N) & Power Generated (W) \\",
                r"\hline",
            ]
            for label, d in zip(labels, drone_bems):
                lines.append(f"{label} & {d.total_thrust_generation:.2f} & {d.total_power_generation:.2f} \\\\")
            lines += [
                r"\hline",
                r"\end{tabular}",
                r"\caption{Thrust and Power for Different Twist Distributions, constant chord: " + ("Yes" if constant_chord else "No") + "}",
                r"\end{table}",
            ]
            print("\n".join(lines))

        print_latex_table(drone_bems, labels)

    compare_twists(constant_chord=False)
    compare_twists(constant_chord=True)
    
    # generate stats for best design with constant chord and constant pitch
    best_constant_chord_design = DroneDesign(
        reference=ingenuity,
        name="Best Constant Chord Design",
        rotor_diameter=quadCopter.rotor_diameter,
        chord=np.full_like(quadCopter.chord, np.mean(quadCopter.chord)),
        N_blades=quadCopter.N_blades,
        N_rotors=quadCopter.N_rotors,
        rpm=quadCopter.rpm,
        N_batteries=quadCopter.N_batteries,
        payload_mass=quadCopter.payload_mass,
        aux_components_mass=quadCopter.aux_components_mass
    )
    best_constant_chord_design.solve_mass_power(P_initial=ingenuity.hover_power, planet=mars)
    best_constant_chord_design.compute_planet_performance(mars)
    
    blade_constant_chord = BladeDesign(drone=best_constant_chord_design,
                                       planet=mars,
                                        c_tip=np.mean(quadCopter.chord))
    blade_constant_chord.set_chord(best_constant_chord_design.chord)
    blade_constant_chord.compute_no_twist()
    blade_constant_chord.set_twist(blade_constant_chord.no_twist)
    
    best_constant_chord_design.bem(blade_constant_chord, linear=False, dimensionless=False)
    best_constant_chord_design.print_stats()
    print(best_constant_chord_design.to_latex_table("Best Design with Constant Chord Distribution"))

if do["Q6"]:
    print("\n################### Q6 ###################\n")
    with open("results/best_designs.pkl", "rb") as f:
        best_optimal = pickle.load(f)

    quadCopter = best_optimal

    V_FORWARD = 10      # m/s — design forward flight speed
    M_PAYLOAD = 2       # kg  — full payload

    DOUBLE_ROTOR_DIAMETER = quadCopter.rotor_diameter * 2
    wingspan_design = DOUBLE_ROTOR_DIAMETER   # design-point wingspan for Re calculation
    AR_wing         = 15                        # aspect ratio — fixed throughout
    wing_chord      = wingspan_design / AR_wing  # FIXED chord, does NOT change with wingspan sweep
    

    # --- Reynolds number and airfoil selection ---
    aircraft_ref = Aircraft(
        reference=ingenuity,
        name="aircraft_ref",
        rotor_diameter=quadCopter.rotor_diameter,
        chord=quadCopter.chord,
        N_blades=quadCopter.N_blades,
        N_rotors=quadCopter.N_rotors,
        wingspan=wingspan_design,
        wing_chord=wing_chord,
        rpm=quadCopter.rpm,
        N_batteries=quadCopter.N_batteries,
        payload_mass=M_PAYLOAD,
        aux_components_mass=1.0
    )
    aircraft_ref.solve_mass_power(P_initial=ingenuity.hover_power, planet=mars, wing_mass=0)
    aircraft_ref.compute_planet_performance(mars)
    aircraft_ref.compute_reynolds(V_FORWARD, mars)
    print(f"Reynolds number for the wings at {V_FORWARD} m/s: Re = {aircraft_ref.Re:.2e}")

    print("We select NACA6409")
    polar_6409_df = xfoil_polar_txt_to_dataframe("data/wing_naca6409_polar.txt")
    polar_6409    = "data/wing_naca6409_polar.txt"
    plot_q4_polars_side_by_side(polar_6409_df, filename="plots/naca6409_polars.png")

    # --- No-wing forward flight sweep (speed sweep) ---
    V_FORWARD_ARRAY = np.linspace(0, 12, 24)
    powers_speed = []
    betas_speed  = []
    for V in V_FORWARD_ARRAY:
        P, beta, T = aircraft_ref.forward_flight_power(V, mars)
        powers_speed.append(P)
        betas_speed.append(beta)

    plot_q6_forward_flight(V_FORWARD_ARRAY, powers_speed, betas_speed,
                           filename="plots/q6_forward_flight.png")

    # --- Wingspan sweep at fixed V=10 m/s, fixed chord ---
    wingspan_array = np.linspace(0, DOUBLE_ROTOR_DIAMETER * 2, 20)
    powers  = []
    weights = []
    wing_weights = []
    wingspan_valid = []

    for ws in wingspan_array:
        ws = float(ws)
        aircraft = Aircraft(
            reference=ingenuity,
            name=f"aircraft_ws_{ws:.2f}",
            rotor_diameter=quadCopter.rotor_diameter,
            chord=quadCopter.chord,
            N_blades=quadCopter.N_blades,
            N_rotors=quadCopter.N_rotors,
            wingspan=ws,
            wing_chord=wing_chord,   # fixed chord throughout sweep
            rpm=quadCopter.rpm,
            N_batteries=quadCopter.N_batteries,
            payload_mass=M_PAYLOAD,
            aux_components_mass=1.0
        )
        wing = WingDesign(
            drone=aircraft,
            planet=mars,
            c_tip=wing_chord,        # same fixed chord
            polar_data=polar_6409
        )
        # wing.compute_optimum_plan_form_and_twist()
        # wing.set_chord(wing.optimum_chord_distribution)
        wing.compute_wing_mass(mars)
        wing.compute_wing_lift_drag(V_FORWARD, mars)

        result = aircraft.solve_mass_power(
            P_initial=ingenuity.hover_power,
            planet=mars,
            wing_mass=wing.mass
        )

        if result is None:
            print(f"  b={ws:.2f}m | FAILED TO CONVERGE")
            continue

        P, beta, T = aircraft.forward_flight_power(V_FORWARD, mars,
                                                    wing_lift=wing.lift,
                                                    wing_drag=wing.drag)
        
        # Only append if P is valid
        if np.isfinite(P):
            powers.append(P)
            weights.append(aircraft.mass)
            wing_weights.append(float(wing.mass) if isinstance(wing.mass, np.ndarray) else wing.mass)
            wingspan_valid.append(ws)
            print(f"  b={ws:.2f}m | wing_mass={float(wing.mass):.3f}kg | "
                  f"lift={wing.lift:.2f}N | drag={wing.drag:.2f}N | "
                  f"total_mass={aircraft.mass:.2f}kg | P={P:.1f}W")
    
    # Use only valid wingspan values for plotting
    plot_q6_wingspan_sweep(np.array(wingspan_valid), powers, weights, V_FORWARD,
                           filename="plots/q6_wingspan_sweep.png")
    
    #extract solution with lowest power
    min_power_idx = np.argmin(powers)
    P_base = powers[min_power_idx]   
    
    results = {
        "density": [],
        "power": [],
        "weight": [],
        "wing_weight": []
    }
    for density in np.linspace(1, aircraft_ref.WING_DENSITY, 100):
        aircraft = Aircraft(
            reference=ingenuity,
            name=f"aircraft_density_{density:.2f}",
            rotor_diameter=quadCopter.rotor_diameter,
            chord=quadCopter.chord,
            N_blades=quadCopter.N_blades,
            N_rotors=quadCopter.N_rotors,
            wingspan=DOUBLE_ROTOR_DIAMETER*2,
            wing_chord=wing_chord,   # fixed chord throughout sweep
            rpm=quadCopter.rpm,
            N_batteries=quadCopter.N_batteries,
            payload_mass=M_PAYLOAD,
            aux_components_mass=1.0
        )
        wing = WingDesign(
            drone=aircraft,
            planet=mars,
            c_tip=aircraft.wing_chord,        # same fixed chord
            polar_data=polar_6409
        )
        wing.WING_DENSITY = density
        wing.compute_wing_mass(mars)
        wing.compute_wing_lift_drag(V_FORWARD, mars)
        result = aircraft.solve_mass_power(
            P_initial=P_base,
            planet=mars,
            wing_mass=wing.mass
        )
        P, beta, T = aircraft.forward_flight_power(V_FORWARD, mars,
                                                    wing_lift=wing.lift,
                                                    wing_drag=wing.drag)
        if np.isfinite(P):
            results["density"].append(density)
            results["power"].append(P)
            results["weight"].append(aircraft.mass)
            results["wing_weight"].append(wing.mass)
    
    
    # plot power vs density
    def plot_q6_density_sweep(results, filename="plots/q6_density_sweep.png"):
        fig, ax1 = plt.subplots(figsize=(8, 6))
        ax1.plot(results["density"], results["power"], label="Total Power [W]", color="blue")
        ax1.axhline(results["power"][-1]*0.9, color="red", linestyle="--", label=f"Target Power (90% of min) = {results['power'][-1]*0.9:.1f} W")
        ax1.set_xlabel("Wing Material Density [kg/m³]")
        ax1.set_ylabel("Total Power [W]", color="blue")
        ax1.tick_params(axis='y', labelcolor="blue")
        ax1.legend(loc="upper left")

        # ax2 = ax1.twinx()
        # ax2.plot(results["density"], results["weight"], label="Total Aircraft Mass [kg]", color="green")
        # ax2.plot(results["density"], results["wing_weight"], label="Wing Mass [kg]", color="orange")
        # ax2.set_ylabel("Mass [kg]", color="green")
        # ax2.tick_params(axis='y', labelcolor="green")
        # ax2.legend(loc="upper right")

        plt.title("Effect of Wing Material Density on Power and Mass at V=10 m/s")
        plt.tight_layout()
        plt.savefig(filename, dpi=150)
        plt.close()
    
    plot_q6_density_sweep(results, filename="plots/q6_density_sweep.png")
    
    # find density where power is just below 10% above the minimum power found in the wingspan sweep
    P_base = results["power"][-1]  # last power value corresponds to the highest density, which should be the lowest power
    target_power = P_base * 0.9
    density_opt = None
    for density, power in zip(results["density"], results["power"]):
        if power <= target_power:
            density_opt = density
        else:
            break
    print(f"Optimal wing material density for power within 10% of minimum: {density_opt:.2f} kg/m³")
    
    print("Final sweep: forward flight power vs speed for multiple fixed beta values")
    
    aircraft = Aircraft(
            reference=ingenuity,
            name=f"aircraft_final_V_{V:.2f}",
            rotor_diameter=quadCopter.rotor_diameter,
            chord=quadCopter.chord,
            N_blades=quadCopter.N_blades,
            N_rotors=quadCopter.N_rotors,
            wingspan=0,
            wing_chord=0,   # fixed chord throughout sweep
            rpm=quadCopter.rpm,
            N_batteries=quadCopter.N_batteries,
            payload_mass=M_PAYLOAD,
            aux_components_mass=1.0)
    
    # wing = WingDesign(
    #         drone=aircraft,
    #         planet=mars,
    #         c_tip=aircraft.wing_chord,        # same fixed chord
    #         polar_data=polar_6409)
    
    # wing.compute_wing_mass(mars)
    
    result = aircraft.solve_mass_power(
            P_initial=P_base,
            planet=mars,
            wing_mass=0.0)

    V_FORWARD_ARRAY = np.linspace(0, 12, 24)
    beta_values = np.linspace(-2, 8, 5)
    
    results = {beta: [] for beta in beta_values}
    
    for beta_deg in beta_values:
        for V in V_FORWARD_ARRAY:
            P, T = aircraft.compute_power_fixed_beta(V_forward=V, beta_deg=beta_deg, planet=mars)
            results[beta_deg].append(P)


    fig, ax = plt.subplots(figsize=(8, 6))

    colors      = plt.cm.tab10(np.linspace(0, 0.6, len(beta_values)))
    linestyles  = ["-", "--", ":", "-.", (0, (3,1,1,1)), (0, (5,1))]

    for (beta_deg, powers), col, ls in zip(results.items(), colors, linestyles):
        ax.plot(V_FORWARD_ARRAY, powers, color=col, linestyle=ls,
                linewidth=2, label=rf"$\beta = {beta_deg:.0f}°$")

    # # Add free-solve trimmed line for reference
    # powers_free, betas_free = [], []
    # for V in V_FORWARD_ARRAY:
    #     P, beta, T = aircraft.forward_flight_power(V, mars)
    #     powers_free.append(P)
    #     betas_free.append(np.degrees(beta))

    # ax.plot(V_FORWARD_ARRAY, powers_free, color="black", linewidth=2.5,
    #         linestyle="--", label=r"Trimmed ($\beta$ from force balance)")

    ax.axvline(x=10, color="gray", linestyle=":", linewidth=1, alpha=0.7)
    ax.set_xlabel("Forward speed [m/s]")
    ax.set_ylabel("Total power [W]")
    ax.set_title("Forward flight power for fixed rotor tilt angles")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("plots/q6_power_vs_speed_multi_beta.png", dpi=150)
    plt.close()
    # def plot_q6_final_speed_sweep(V_array, powers, betas_deg, filename="plots/q6_final_speed_sweep.png"):
    #     fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    #     # Power
    #     ax1.plot(V_array, powers, color="tab:blue", linewidth=2)
    #     ax1.axvline(x=10, color="gray", linestyle="--", linewidth=1, label="Design point (10 m/s)")
    #     ax1.set_xlabel("Forward speed [m/s]")
    #     ax1.set_ylabel("Total power [W]")
    #     ax1.set_title("Power vs. speed — final design (no wings)")
    #     ax1.legend()
    #     ax1.grid(True, alpha=0.3)

    #     # Beta
    #     ax2.plot(V_array, betas_deg, color="tab:orange", linewidth=2)
    #     ax2.axvline(x=10, color="gray", linestyle="--", linewidth=1, label="Design point (10 m/s)")
    #     ax2.set_xlabel("Forward speed [m/s]")
    #     ax2.set_ylabel(r"Rotor tilt angle $\beta$ [°]")
    #     ax2.set_title(r"Rotor tilt $\beta$ vs. speed — final design")
    #     ax2.legend()
    #     ax2.grid(True, alpha=0.3)

    #     plt.tight_layout()
    #     plt.savefig(filename, dpi=150)
    #     plt.close()
    
    # plot_q6_final_speed_sweep(V_FORWARD_ARRAY, powers_final, betas_final,
    #                          filename="plots/q6_final_speed_sweep.png")
    print("\nQ6 DONE\n")

if do["Q6_2d_grid_search"]:
    # --- 2D sweep: wingspan × chord ---
    b_array = np.linspace(0.01, DOUBLE_ROTOR_DIAMETER * 2, 15)   # wingspan [m]
    c_array = np.linspace(0.05, 0.60, 15)                         # chord [m], physically reasonable range

    power_grid  = np.full((len(b_array), len(c_array)), np.nan)
    weight_grid = np.full((len(b_array), len(c_array)), np.nan)

    for i, ws in enumerate(b_array):
        for j, c in enumerate(c_array):
            aircraft = Aircraft(
                reference=ingenuity,
                name=f"aircraft_b{ws:.2f}_c{c:.3f}",
                rotor_diameter=quadCopter.rotor_diameter,
                chord=quadCopter.chord,
                N_blades=quadCopter.N_blades,
                N_rotors=quadCopter.N_rotors,
                wingspan=ws,
                wing_chord=c,
                rpm=quadCopter.rpm,
                N_batteries=quadCopter.N_batteries,
                payload_mass=M_PAYLOAD,
                aux_components_mass=1.0
            )
            wing = WingDesign(drone=aircraft, planet=mars, c_tip=c, polar_data=polar_6409)
            wing.compute_wing_mass(mars)
            wing.compute_wing_lift_drag(V_FORWARD, mars)

            result = aircraft.solve_mass_power(
                P_initial=ingenuity.hover_power, planet=mars, wing_mass=wing.mass
            )
            if result is None:
                continue

            P, beta, T = aircraft.forward_flight_power(V_FORWARD, mars,
                                                        wing_lift=wing.lift,
                                                        wing_drag=wing.drag)
            if np.isfinite(P) and aircraft.mass < 500:
                power_grid[i, j]  = P
                weight_grid[i, j] = aircraft.mass

    # Find global optimum
    opt_idx = np.unravel_index(np.nanargmin(power_grid), power_grid.shape)
    b_opt   = b_array[opt_idx[0]]
    c_opt   = c_array[opt_idx[1]]
    P_opt   = power_grid[opt_idx]
    print(f"Optimal: b={b_opt:.3f} m, c={c_opt:.3f} m, AR={b_opt/c_opt:.1f}, P={P_opt:.1f} W")
    
    def plot_q6_2d_sweep(b_array, c_array, power_grid, b_opt, c_opt,
                     filename="plots/q6_2d_sweep.png"):
        fig, ax = plt.subplots(figsize=(8, 6))
        cf = ax.contourf(c_array, b_array, power_grid, levels=20, cmap="viridis_r")
        plt.colorbar(cf, ax=ax, label="Total power [W]")
        ax.contour(c_array, b_array, power_grid, levels=20, colors="white", linewidths=0.5, alpha=0.4)
        ax.scatter([c_opt], [b_opt], color="red", s=100, zorder=5, label=f"Optimum (b={b_opt:.2f}m, c={c_opt:.2f}m)")
        ax.set_xlabel("Wing chord $c$ [m]")
        ax.set_ylabel("Wingspan $b$ [m]")
        ax.set_title(f"Forward flight power [W] at $V$ = {V_FORWARD} m/s")
        ax.legend()
        plt.tight_layout()
        plt.savefig(filename, dpi=150)
        plt.close()
        
    plot_q6_2d_sweep(b_array, c_array, power_grid, b_opt, c_opt,
                     filename="plots/q6_2d_sweep.png")