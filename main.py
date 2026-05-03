import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import numpy as np
from planet import Planet
from drone import Drone, Ingenuity, DroneDesign
from blade_design import BladeDesign
from plot_funcs import (plot_big_grid_search,
                        plot_narrow_grid_search,
                        plot_q3_battery_sweep,
                        plot_weight_distribution_pie,
                        plot_q4_polars_side_by_side,
                        plot_q5_twist_subplots,
                        plot_q5_chord_distribution
                        )
from print_funcs import (print_q3_optimums,
                         print_q2_comparison_latex,
                         print_q4_reynolds_latex
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
    "Q3": False,
    "Q4": False,
    "Q5": False,
    "Q5_bem": True,
    "bem_template": True,
    "bem": False,
    "bem_master": False,
    "Q6": False
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
    
    ingenuity = Ingenuity(
        name="Ingenuity",
        mass=1.8,
        rotor_diameter=1.2,
        chord=C_MEAN,
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

    
    for N_blades in N_blade_array:
        for diameter in diameter_array:
        
            # NEW DRONE DESIGN
            dualCopter = DroneDesign(
                reference=ingenuity,
                name="Dual Copter",
                rotor_diameter=diameter,
                chord=C_MEAN,
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
                chord=C_MEAN,
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
    PRINT("\n################### Q2_NARROW_GRID_SEARCH ###################\n")
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
                chord=C_MEAN,
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
                chord=C_MEAN,
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
                chord=C_MEAN,
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
                chord=C_MEAN,
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

if do["Q3"]:
    print("\n################### Q3 ###################\n")
    print("\nRunning Q3 and determining optimum number of batteries\n")

    # Requires best_dual / best_quad from Q2_FINAL_GRID_SEARCH
    if "best_dual" not in locals() or "best_quad" not in locals():
        raise RuntimeError("Run Q2_FINAL_GRID_SEARCH first to define best_dual and best_quad.")

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
        dualCopter = DroneDesign(
            reference=ingenuity,
            name="Dual Copter",
            rotor_diameter=best_dual.rotor_diameter,
            chord=C_MEAN,
            N_blades=best_dual.N_blades,
            N_rotors=2,
            rpm=2800,
            N_batteries=int(n_total),
            payload_mass=0,
            aux_components_mass=1.0
        )

        quadCopter = DroneDesign(
            reference=ingenuity,
            name="Quad Copter",
            rotor_diameter=best_quad.rotor_diameter,
            chord=C_MEAN,
            N_blades=best_quad.N_blades,
            N_rotors=4,
            rpm=2800,
            N_batteries=int(n_total),
            payload_mass=0,
            aux_components_mass=1.0
        )

        for drone in (dualCopter, quadCopter):
            result = drone.solve_mass_power(P_initial, mars)
            if result is not None:
                results[drone.name]["N_batt"].append(int(n_total))
                results[drone.name]["time_min"].append(drone.total_hover_time / 60.0)

    plot_q3_battery_sweep(
        results=results,
        N_batt_max_2kg=N_batt_max_2kg,
        filename="plots/q3_flight_time_vs_N_batteries.png"
    )

    print_q3_optimums(
        results=results,
        N_batt_max_2kg=N_batt_max_2kg
    )

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
    print("\nRunning Q4 and calculating Reynolds numbers for Earth and Mars\n")
    
    earth = Planet("Earth", g=9.81, rho=1.225, viscosity=1.8e-5)
    mars = Planet("Mars", g=3.72, rho=0.01503, viscosity=1.3e-5)
    
    def compute_reynolds(planet, drone, r_fraction):
        # Calculate velocity at r_fraction of rotor radius
        R = drone.rotor_radius
        c = drone.chord
        rpm = drone.rpm
        r = r_fraction * R
        V = rpm * 2 * np.pi * r / 60.0
        
        # Calculate Reynolds number
        Re = planet.rho * V * c / planet.viscosity
        return Re
    
    planets = [earth, mars]
    drones = [best_dual, best_quad]
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

    
    
    print("\nQ4 DONE\n")
    
if do["Q5"]:
    print("\n################### Q5 ###################\n")

    print("\nRunning Q5 and designing blades based on optimum design\n")
    
    # load reference data from ingenuity nasa report for twist distribution and chord distribution    
    blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
    
    # We can use the tip chord length from the reference data to more accurately compute the chord distribution
    C_TIP = blade_geometry_nasa['chord'][-13]  # Assuming the first entry corresponds to the tip chord
    print(f"Using tip chord length from NASA data for blade design: {C_TIP:.4f} m")

    
    drones = [best_dual, best_quad]
    
    print("initialize best dual")
    dual_blade_design = BladeDesign(drone=best_dual, planet=mars)
    print("initialize best quad")
    quad_blade_design = BladeDesign(drone=best_quad, planet=mars)
    
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
    print("\nQ5 DONE\n")

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
    
if do["Q5_bem"]:
    print("\nRunning Q5 BEM and designing blades based on optimum design\n")

    # load reference data from ingenuity nasa report for twist distribution and chord distribution    
    blade_geometry_nasa = read_blade_geometry_nasa("data/ingenuity_nasa_blade_geometry.txt")
    NO_BLADE_ELEMENTS = len(blade_geometry_nasa['y'])

    # so we found out with "bem" and "bem_master" that our chord is likely too small.
    # we also found out that our bem likely has some bugs, since we cant even produce enough torque for ingenuity
    # anyways, lets try to iterate through chord lengths and see if we can get enough torque for our design, with the twist we have already computed
    
    # because we want to use the full chord distribution now, we need to generate a drone instance, so we can generate a blade instance
    drone = DroneDesign(
                    reference=ingenuity,
                    name="bemDrone",
                    rotor_diameter=0.76,
                    chord=C_MEAN,
                    N_blades=2,
                    N_rotors=2,
                    rpm=2800,
                    N_batteries=48,
                    payload_mass=0.0,
                    aux_components_mass=1.0
                )
    # with starting point in this example drone instance, we can iterate over a c_tip array, to first generate a chord distributtion, and then generate a drone based on that
    # We can use the tip chord length from the reference data as the shortest chord
    C_TIP = blade_geometry_nasa['chord'][-13]  # Assuming the first entry corresponds to the tip chord


    
    
    for chord in np.linspace(C_TIP, 0.2, 10):
        blade = BladeDesign(drone=drone, planet=mars, c_tip=chord, no_blade_elements=NO_BLADE_ELEMENTS)
        blade.compute_optimum_plan_form_and_twist()
        blade.set_chord(blade.optimum_chord_distribution)
        drone = DroneDesign(
                    reference=ingenuity,
                    name="bemDrone",
                    rotor_diameter=0.76,
                    chord=blade.chord,
                    N_blades=2,
                    N_rotors=2,
                    rpm=2800,
                    N_batteries=48,
                    payload_mass=0.0,
                    aux_components_mass=1.0
                )
        
        drone.solve_mass_power(P_initial=ingenuity.hover_power, planet=mars)
        
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