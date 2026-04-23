import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import numpy as np
from planet import Planet
from drone import Drone
from plot_funcs import plot_big_grid_search, plot_narrow_grid_search

#TODO DO THE FINAL GRID SEARCH

do = {
    "Q1": True,
    "Q2_test_one_design_dual": False,
    "Q2_test_one_design_dual_help_func": False,
    "Q2_test_one_design_quad": False,
    "Q2_test_one_design_quad_help_func": False,
    "Q2_BIG_GRID_SEARCH": False,
    "Q2_NARROW_GRID_SEARCH": True,
    "Q2_FINAL_GRID_SEARCH": False,
    "Q2_with_help_func": False,
    "Q3": False,
    "Q4": False,
    "Q5": False
    }

if do["Q1"]:
    """ 
    Data from pdf file:
   
    
        Components using battery power: rotors, computer, control motors, and heating systems
        
        Assume 2800 rpm for task 1 calculations
        Assume Cd0 = 0.02 for rotor loss calculations
            
        Evaluate dualcopter vs quadcopter drone designs
        
        Find the weight, optimum radius of propellers, number of blades and total power consumption
    
    INGENUITY REFERENCE DESIGN:
    
        WEIGHT DISTRIBUTION OF INGENUITY
        
            Two rotors = 70 g = 0.07 kg
            Battery pack = 280 g = 0.28 kg
            Propulsion and control motors = 250 g = 0.25 kg
            Fuselage = 300 g = 0.3 kg
            Computer and other components = 900 g = 0.9 kg
            Total = 1800 g = 1.8 kg
            
            weight per blade = 70 g / (2 rotors * 2 blades) 
            
            Ingenuity payload = 0 kg
        
        ROTOR DESIGN
            two contra-rotating rotors
                Rotor diameter = 1.2 m
            From Balaram et al., 2018 we get a constant chord of about 55mm = 0.055 m
                c_mean = 0.055 m
        
        ROTATIONAL SPEED
            rpm: 2800 rpm
        
        POWER CONSUMPTION
            Under peak load batteries provided: 510 W
            During hover batteries provided: 360 W
        
        BATTERY PACK
            battery pack energy = 10 Wh
            batteries per pack = 6
            
    NEW DESIGN CONSTRAINTS:
    
        New design: payload = 2 kg
        
        battery pack mass = 500 g = 0.5 kg
        battery pack energy = 20 Wh 
        
        The blades of your propellers are expected to have the same average weight per unit of 
        length as those in Ingenuity.
        weight of one propeller is:
        
            weight per propeller: (70g/4) * Nblades * (R/RIngenuity). 
        
        The weight of the propulsion and control motors of each rotor scales linearly with the 
        power in hover of the rotor (this requires your estimation from task 1). 
        For example, if the propeller of your new design requires a power Pnew, the weight of the motor driving that 
        propeller is 
        
            Pnew = (250g/2)*Pnew/Pingenuity
        
        where 250g/2 is the weight of the propulsion motors in Ingenuity divided by its number of rotors,
        and Pingenuity is the average power consumption per rotor in Ingenuity (estimated in task 1).
        
            computer and other components mass = 1000 g = 1 kg
        
        The weight of the fuselage scales linearly with the weight of the rest of the aircraft 
        (including the payload). 
        So the mass of the fuselage is
        
            Mfus = M_no_fuselage*(Mfus_ingenuity/M_no_fuselage_ingenuity) 
    
    For each design option (two rotors and quadcopter), compare the following in a table: 
        1. Radius of the rotor 
        2. Number of blades 
        3. Total power consumption from the rotors (total power required to produce the desired 
        thrust)  
        4. Aircrafts total mass with payload 
        5. Flight time in hover 
        
        Choose what you believe is the best design and provide the weight distribution estimation (pie chart)
        
    """
    print("\nRunning Q1 and generating performance metrics for Ingenuity reference design on planet: Mars\n")
    mars = Planet("Mars", g=3.712)
    
    ingenuity = Drone(
        name="Ingenuity",
        mass=1.8, fuselage_mass=0.3, payload_mass=0, battery_mass=0.28,
        aux_components_mass=0.9, motor_mass=0.25, rotor_mass=0.07,
        N_batteries=6, total_battery_capacity=10, 
        rotor_diameter=1.2, chord=0.055, N_blades=2, N_rotors=2,
        rpm=2800,
        peak_power=510, avg_power=360,
        C_D0=0.02, gamma=1.15)

    ingenuity.planet_performance(mars)

    print(f"\n{ingenuity.name}: Total Power = {ingenuity.hover_power:.2f} W, Total Mass = {ingenuity.mass:.2f} kg, Flight Time = {ingenuity.total_hover_time:.2f} s")
    #TODO LOOK INTO FORMULAS (SEE Q1_class.py)
    print("\nQ1 DONE \n")

if do["Q2_BIG_GRID_SEARCH"]:
    
    # DESIGN CONSTRAINTS
    PAYLOAD_MASS = 2.0 # kg
    BATTERY_PACK_MASS = 0.5 # kg
    AUX_COMPONENTS_MASS = 1.0 # kg
    
    # ROTOR DIAMETER GRID SEARCH ARRAY
    # From 1/3 ingenuity to 4x ingenuity as a starting point
    diameter_array = np.linspace(ingenuity.rotor_diameter / 3, 4 * ingenuity.rotor_diameter, 10)
    
    # NUMBER OF BLADES TO TEST
    N_blade_array = [2, 3, 4, 5]
    
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    # Initial guess for solver
    P_initial = max(ingenuity.hover_power, 1.0)  

    
    for N_blades in N_blade_array:
        for diameter in diameter_array:
        
            # NEW DRONE DESIGN
            dualCopter = Drone(
                name="Dual Copter",
                mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
                aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
                N_batteries=6, total_battery_capacity=10, 
                rotor_diameter=diameter, chord=0.055, N_blades=N_blades, N_rotors=2,
                rpm=2800,
                peak_power=510, avg_power=360,
                C_D0=0.02, gamma=1.15)
            
            quadCopter = Drone(
                name="Quad Copter",
                mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
                aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
                N_batteries=6, total_battery_capacity=10,
                rotor_diameter=diameter, chord=0.055, N_blades=N_blades, N_rotors=4,
                rpm=2800,
                peak_power=510, avg_power=360,
                C_D0=0.02, gamma=1.15)
            
            drone_designs = [dualCopter, quadCopter]
            
            for drone in drone_designs:
                # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
                result = drone.solve_mass_power(P_initial, ingenuity, mars)
                if result is not None:
                    # Store the design in the list
                    converged_drone_designs.append(result)
                    # Print results for each converged design
                    print(f"\n{drone.name}: Total Power = {drone.hover_power:.2f} W, Total Mass = {drone.mass:.2f} kg, Flight Time = {drone.total_hover_time:.2f} s")

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
    
    # DESIGN CONSTRAINTS
    PAYLOAD_MASS = 2.0 # kg
    BATTERY_PACK_MASS = 0.5 # kg
    AUX_COMPONENTS_MASS = 1.0 # kg
    
    # ROTOR DIAMETER GRID SEARCH ARRAY
    # Now that we have already made a big grid search, we can narrow down the range of rotor diameters to test based on the results of the big grid search.
    # It was very clear that 2 blades was the best option, so we will only test 2 blades in the narrow grid search.
    # Diameter wise, 
    ROTOR_LOWER_BOUND = 0.44*2
    ROTOR_UPPER_BOUND = 0.93*2
    diameter_array = np.linspace(ROTOR_LOWER_BOUND, ROTOR_UPPER_BOUND, 10)
    
    # ROTOR_LOWER_BOUND_QUAD = 0.5*2
    # ROTOR_UPPER_BOUND_QUAD = 1.5*2
    # diameter_array_quad = np.linspace(ROTOR_LOWER_BOUND_QUAD, ROTOR_UPPER_BOUND_QUAD, 10)
    
    # NUMBER OF BLADES TO TEST
    N_blade_optimum = 2
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    # Initial guess for solver
    P_initial = max(ingenuity.hover_power, 1.0)  

    
    for diameter in diameter_array:
    
        # NEW DRONE DESIGN
        dualCopter = Drone(
            name="Dual Copter",
            mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
            aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
            N_batteries=6, total_battery_capacity=10, 
            rotor_diameter=diameter, chord=0.055, N_blades=N_blade_optimum, N_rotors=2,
            rpm=2800,
            peak_power=510, avg_power=360,
            C_D0=0.02, gamma=1.15)
        
        quadCopter = Drone(
            name="Quad Copter",
            mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
            aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
            N_batteries=6, total_battery_capacity=10,
            rotor_diameter=diameter, chord=0.055, N_blades=N_blade_optimum, N_rotors=4,
            rpm=2800,
            peak_power=510, avg_power=360,
            C_D0=0.02, gamma=1.15)
        
        drone_designs = [dualCopter, quadCopter]
        
        for drone in drone_designs:
            # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
            result = drone.solve_mass_power(P_initial, ingenuity, mars)
            if result is not None:
                # Store the design in the list
                converged_drone_designs.append(result)
                # Print results for each converged design
                print(f"\n{drone.name}: Total Power = {drone.hover_power:.2f} W, Total Mass = {drone.mass:.2f} kg, Flight Time = {drone.total_hover_time:.2f} s")

            else:
                print(f"Skipping {drone.name} at R={drone.rotor_radius:.2f} m, blades={drone.N_blades}")

    plot_narrow_grid_search(
        converged_drone_designs=converged_drone_designs,
        filename='plots/narrow_grid_search_performance.png',
        fig_title='Narrow Grid Search — 2 Blades'
    )

    
    print("\nQ2 NARROW GRID SEARCH DONE \n")

if do["Q2_FINAL_GRID_SEARCH"]:
    
    # DESIGN CONSTRAINTS
    PAYLOAD_MASS = 2.0 # kg
    BATTERY_PACK_MASS = 0.5 # kg
    AUX_COMPONENTS_MASS = 1.0 # kg
    
    # ROTOR DIAMETER GRID SEARCH ARRAY
    # Now that have narrowed in the rotor radius, we can do a final grid search with a finer resolution around the best performing rotor radius from the narrow grid search.
    #TODO UPDATE BASED ON PLOT WHEN ALL FORMULAS ARE CHECKED THROUGH
    # multiply by 2 since diameter = 2*radius
    ROTOR_LOWER_BOUND_DUAL = 0.75*2
    ROTOR_UPPER_BOUND_DUAL = 1.25*2
    diameter_array_dual = np.linspace(ROTOR_LOWER_BOUND_DUAL, ROTOR_UPPER_BOUND_DUAL, 10)
    
    ROTOR_LOWER_BOUND_QUAD = 0.5*2
    ROTOR_UPPER_BOUND_QUAD = 1.5*2
    diameter_array_quad = np.linspace(ROTOR_LOWER_BOUND_QUAD, ROTOR_UPPER_BOUND_QUAD, 10)
    
    # NUMBER OF BLADES TO TEST
    N_blade_optimum = 2
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    for i in range(len(diameter_array_dual)):
    
        # NEW DRONE DESIGN
        dualCopter = Drone(
            name="Dual Copter",
            mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
            aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
            N_batteries=6, total_battery_capacity=10, 
            rotor_diameter=diameter_array_dual[i], chord=0.055, N_blades=N_blade_optimum, N_rotors=2,
            rpm=2800,
            peak_power=510, avg_power=360,
            C_D0=0.02, gamma=1.15)
        
        quadCopter = Drone(
            name="Quad Copter",
            mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
            aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
            N_batteries=6, total_battery_capacity=10,
            rotor_diameter=diameter_array_quad[i], chord=0.055, N_blades=N_blade_optimum, N_rotors=4,
            rpm=2800,
            peak_power=510, avg_power=360,
            C_D0=0.02, gamma=1.15)
        
        drone_designs = [dualCopter, quadCopter]
        
        for drone in drone_designs:
            # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
            result = drone.solve_mass_power(P_initial, ingenuity, mars)
            if result is not None:
                # Store the design in the list
                converged_drone_designs.append(result)
                # Print results for each converged design
                print(f"\n{drone.name}: Total Power = {drone.hover_power:.2f} W, Total Mass = {drone.mass:.2f} kg, Flight Time = {drone.total_hover_time:.2f} s")

            else:
                print(f"Skipping {drone.name} at R={drone.rotor_radius:.2f} m, blades={drone.N_blades}")

    plot_narrow_grid_search(
        converged_drone_designs=converged_drone_designs,
        filename='plots/final_grid_search_performance.png',
        fig_title='Final Grid Search — 2 Blades'
    )
    
    # Choose best design based on flight time in hover (since we want to maximize flight time for a given payload)
    # choose best dualcopter and best quadcopter design
    best_dual = max([d for d in converged_drone_designs if d.name == "Dual Copter"], key=lambda d: d.total_hover_time)
    best_quad = max([d for d in converged_drone_designs if d.name == "Quad Copter"], key=lambda d: d.total_hover_time)
    
    # Save the configuration (radius and blades) in variables
    best_dual_config = (best_dual.rotor_radius, best_dual.N_blades)
    best_quad_config = (best_quad.rotor_radius, best_quad.N_blades)
    
    # print results for best designs
    print(f"\nBest Dual Copter: Rotor Radius = {best_dual.rotor_radius:.2f} m, Blades = {best_dual.N_blades}, Flight Time = {best_dual.total_hover_time:.2f} s")
    print(f"Best Quad Copter: Rotor Radius = {best_quad.rotor_radius:.2f} m, Blades = {best_quad.N_blades}, Flight Time = {best_quad.total_hover_time:.2f} s")
    
    print("\nQ2 FINAL GRID SEARCH DONE \n")

