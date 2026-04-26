""" old scripts to test one design only"""
if do["Q2_test_one_design_dual"]:
    print("\nQ2 TEST ONE DESIGN \n")
    # DESIGN CONSTRAINTS
    PAYLOAD_MASS = 2.0 # kg
    BATTERY_PACK_MASS = 0.5 # kg
    AUX_COMPONENTS_MASS = 1.0 # kg
    
    # CONFIG
    DIAMETER = 1
    N_BLADE_OPTIMUM = 2
    
    # NEW DRONE DESIGN
    dualCopter = Drone(
        name="Dual Copter",
        mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
        aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
        N_batteries=6, total_battery_capacity=10, 
        rotor_diameter=DIAMETER, chord=0.055, N_blades=N_BLADE_OPTIMUM, N_rotors=2,
        rpm=2800,
        peak_power=510, avg_power=360,
        C_D0=0.02, gamma=1.15)
    
        
        
        
        
    # compute the mass of one rotor (Scale linearly with rotor diameter (same mass per unit length as ingenuity))
    dualCopter.rotor_mass = ingenuity.blade_mass * dualCopter.N_blades * dualCopter.N_rotors * dualCopter.rotor_diameter / ingenuity.rotor_diameter
            
    # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
    
    P_drone = max(ingenuity.hover_power, 1.0)  # better initial guess
    tol = 1e-4 # convergence threshold [W]
    max_iter = 1000
    alpha = 0.5 # relaxation factor
                
    for i in range(max_iter):
        
        # 1. Estimate mass components that depend on P_drone
        dualCopter.motor_mass = dualCopter.N_rotors * (ingenuity.motor_mass/ ingenuity.N_rotors) * (P_drone / ingenuity.hover_power) # kg

        # 2. Compute the mass of the drone without fuselage
        dualCopter.mass_no_fuselage = dualCopter.payload_mass + dualCopter.battery_mass + dualCopter.aux_components_mass + dualCopter.rotor_mass + dualCopter.motor_mass
        
        # 3. compute the mass of the fuselage, by the linear relationship with ingenuity
        dualCopter.fuselage_mass = dualCopter.mass_no_fuselage * (ingenuity.fuselage_mass / ingenuity.mass_no_fuselage)
        
        # 4. Compute the total mass of the drone
        dualCopter.mass = dualCopter.mass_no_fuselage + dualCopter.fuselage_mass            
        
        # 5. Compute the hover power for this design
        dualCopter.planet_performance(mars)
        P_new = dualCopter.hover_power
        
        # 6. Check convergence
        if abs(P_new - P_drone) < tol:
            # print(f"Converged in {i+1} iterations")
            break
        
        # 7. Relaxed update — blend old and new estimate to avoid oscillation
        P_drone = alpha * P_new + (1 - alpha) * P_drone
            

            
    # Print results
    print(f"\n{dualCopter.name}: Total Power = {dualCopter.hover_power:.2f} W, Total Mass = {dualCopter.mass:.2f} kg, Flight Time = {dualCopter.total_hover_time:.2f} s")
    print("\nQ2 TEST ONE DESIGN FINISHED \n")
            
if do["Q2_test_one_design_dual_help_func"]:

    print("\nQ2 TEST ONE DESIGN WITH HELPER FUNC \n")

    # DESIGN CONSTRAINTS
    PAYLOAD_MASS = 2.0 # kg
    BATTERY_PACK_MASS = 0.5 # kg
    AUX_COMPONENTS_MASS = 1.0 # kg
    
    # CONFIG
    DIAMETER = 1
    N_BLADE_OPTIMUM = 2
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    
    # NEW DRONE DESIGN
    dualCopter = Drone(
        name="Dual Copter",
        mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
        aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
        N_batteries=6, total_battery_capacity=10, 
        rotor_diameter=DIAMETER, chord=0.055, N_blades=N_BLADE_OPTIMUM, N_rotors=2,
        rpm=2800,
        peak_power=510, avg_power=360,
        C_D0=0.02, gamma=1.15)
    
    P_initial = max(ingenuity.hover_power, 1.0)  # better initial guess

    # compute the mass of one rotor (Scale linearly with rotor diameter (same mass per unit length as ingenuity))
    # dualCopter.rotor_mass = ingenuity.blade_mass * dualCopter.N_blades * dualCopter.N_rotors * dualCopter.rotor_diameter / ingenuity.rotor_diameter
                
    # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
    dualCopter.solve_mass_power(P_initial=P_initial, ingenuity=ingenuity, planet=mars, tol=1e-4, max_iter=1000, alpha=0.5)
            
    # Print results
    print(f"\n{dualCopter.name}: Total Power = {dualCopter.hover_power:.2f} W, Total Mass = {dualCopter.mass:.2f} kg, Flight Time = {dualCopter.total_hover_time:.2f} s")
    
    print("\nQ2 TEST ONE DESIGN WITH HELPER FUNC FINISHED\n")

if do["Q2_test_one_design_quad"]:
    print("\nQ2 TEST ONE DESIGN \n")
    # DESIGN CONSTRAINTS
    PAYLOAD_MASS = 2.0 # kg
    BATTERY_PACK_MASS = 0.5 # kg
    AUX_COMPONENTS_MASS = 1.0 # kg
    
    # CONFIG
    DIAMETER = 1
    N_BLADE_OPTIMUM = 2
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    
    # NEW DRONE DESIGN
    quadCopter = Drone(
        name="Quad Copter",
        mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
        aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
        N_batteries=6, total_battery_capacity=10, 
        rotor_diameter=DIAMETER, chord=0.055, N_blades=N_BLADE_OPTIMUM, N_rotors=4,
        rpm=2800,
        peak_power=510, avg_power=360,
        C_D0=0.02, gamma=1.15)
    
        
        
        
        
    # compute the mass of one rotor (Scale linearly with rotor diameter (same mass per unit length as ingenuity))
    quadCopter.rotor_mass = ingenuity.blade_mass * quadCopter.N_blades * quadCopter.N_rotors * quadCopter.rotor_diameter / ingenuity.rotor_diameter
            
    # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
    
    P_drone = max(ingenuity.hover_power, 1.0)  # better initial guess
    tol = 1e-4 # convergence threshold [W]
    max_iter = 1000
    alpha = 0.5 # relaxation factor
                
    for i in range(max_iter):
        
        # 1. Estimate mass components that depend on P_drone
        quadCopter.motor_mass = quadCopter.N_rotors * (ingenuity.motor_mass/ ingenuity.N_rotors) * (P_drone / ingenuity.hover_power) # kg

        # 2. Compute the mass of the drone without fuselage
        quadCopter.mass_no_fuselage = quadCopter.payload_mass + quadCopter.battery_mass + quadCopter.aux_components_mass + quadCopter.rotor_mass + quadCopter.motor_mass
        
        # 3. compute the mass of the fuselage, by the linear relationship with ingenuity
        quadCopter.fuselage_mass = quadCopter.mass_no_fuselage * (ingenuity.fuselage_mass / ingenuity.mass_no_fuselage)
        
        # 4. Compute the total mass of the drone
        quadCopter.mass = quadCopter.mass_no_fuselage + quadCopter.fuselage_mass            
        
        # 5. Compute the hover power for this design
        quadCopter.planet_performance(mars)
        P_new = quadCopter.hover_power
        
        # 6. Check convergence
        if abs(P_new - P_drone) < tol:
            # print(f"Converged in {i+1} iterations")
            break
        
        # 7. Relaxed update — blend old and new estimate to avoid oscillation
        P_drone = alpha * P_new + (1 - alpha) * P_drone
            

            
    # Print results
    print(f"\n{quadCopter.name}: Total Power = {quadCopter.hover_power:.2f} W, Total Mass = {quadCopter.mass:.2f} kg, Flight Time = {quadCopter.total_hover_time:.2f} s")
    print("\nQ2 TEST ONE DESIGN FINISHED \n")
            
if do["Q2_test_one_design_quad_help_func"]:

    print("\nQ2 TEST ONE DESIGN WITH HELPER FUNC \n")

    # DESIGN CONSTRAINTS
    PAYLOAD_MASS = 2.0 # kg
    BATTERY_PACK_MASS = 0.5 # kg
    AUX_COMPONENTS_MASS = 1.0 # kg
    
    # CONFIG
    DIAMETER = 1
    N_BLADE_OPTIMUM = 2
    # CREATE LIST TO STORE EACH DRONE DESIGN WHEN THE ITERATIVE METHOD CONVERGES
    converged_drone_designs = []
    
    
    # NEW DRONE DESIGN
    quadCopter = Drone(
        name="Quad Copter",
        mass=0, fuselage_mass=0, payload_mass=PAYLOAD_MASS, battery_mass=BATTERY_PACK_MASS, 
        aux_components_mass=AUX_COMPONENTS_MASS, motor_mass=0.25, rotor_mass=0,
        N_batteries=6, total_battery_capacity=10, 
        rotor_diameter=DIAMETER, chord=0.055, N_blades=N_BLADE_OPTIMUM, N_rotors=4,
        rpm=2800,
        peak_power=510, avg_power=360,
        C_D0=0.02, gamma=1.15)
    

    #NOTE THERE IS AN ISSUE WITH THE QUADCOPTER, NOT THE DUAL COPTER, WHAT COULD CAUSE THIS?
    
    P_initial = max(ingenuity.hover_power, 1.0)  # better initial guess
                
    # solve for the total power and mass of the drone designs iteratively (since power depends on mass and mass depends on power)
    quadCopter.solve_mass_power(P_initial=P_initial, ingenuity=ingenuity, planet=mars, tol=1e-4, max_iter=1000, alpha=0.5)
            
    # Print results
    print(f"\n{quadCopter.name}: Total Power = {quadCopter.hover_power:.2f} W, Total Mass = {quadCopter.mass:.2f} kg, Flight Time = {quadCopter.total_hover_time:.2f} s")
    
    print("\nQ2 TEST ONE DESIGN WITH HELPER FUNC FINISHED\n")

