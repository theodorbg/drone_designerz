import subprocess
import numpy as np
import os

def run_xfoil_polar(airfoil: str, Re: float, alpha_start: float = -10, alpha_end: float = 20,
                    alpha_step: float = 0.5, n_iter: int = 200, output_file: str = "data/wing_polar.txt"):
    """
    Run XFOIL to generate a polar for a given airfoil at a given Reynolds number.
    
    Args:
        airfoil    : NACA designation e.g. "6409" or path to .dat file
        Re         : Reynolds number
        alpha_start: start angle of attack [deg]
        alpha_end  : end angle of attack [deg]
        alpha_step : step size [deg]
        n_iter     : max iterations (use 200+ for low Re)
        output_file: path to save polar
    """
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    # Build XFOIL command sequence
    commands = [
        f"NACA {airfoil}",   # load airfoil
        "PANE",              # repanel
        "OPER",              # operating point menu
        f"VISC {Re:.0f}",    # viscous mode at Re
        f"ITER {n_iter}",    # max iterations
        "PACC",              # start polar accumulation
        output_file,         # save to file
        "",                  # no dump file
        f"ASEQ {alpha_start} {alpha_end} {alpha_step}",  # alpha sweep
        "PACC",              # stop accumulation
        "",
        "QUIT"
    ]

    xfoil_input = "\n".join(commands) + "\n"

    XFOIL_PATH = r"C:\Users\tgilh\OneDrive\Desktop\XFOIL6.99\xfoil.exe"   # adjust to your actual location

    result = subprocess.run(
        [XFOIL_PATH],
        input=xfoil_input,
        capture_output=True,
        text=True,
        timeout=120
    )

    if result.returncode != 0:
        print("XFOIL stderr:", result.stderr)
    else:
        print(f"Polar saved to {output_file}")

    return result


def parse_xfoil_polar(filepath: str) -> dict:
    """
    Parse XFOIL polar output file into arrays.
    Returns dict with keys: alpha, Cl, Cd, Cdp, Cm, Top_Xtr, Bot_Xtr
    """
    alpha, Cl, Cd, Cm = [], [], [], []

    with open(filepath, "r") as f:
        lines = f.readlines()

    # Skip header lines (first 12 lines in XFOIL output)
    data_started = False
    for line in lines:
        if "alpha" in line.lower() and "CL" in line:
            data_started = True
            continue
        if "----" in line:
            continue
        if data_started and line.strip():
            vals = line.split()
            if len(vals) >= 4:
                try:
                    alpha.append(float(vals[0]))
                    Cl.append(float(vals[1]))
                    Cd.append(float(vals[2]))
                    Cm.append(float(vals[4]))
                except ValueError:
                    continue

    return {
        "alpha": np.array(alpha),
        "Cl":    np.array(Cl),
        "Cd":    np.array(Cd),
        "Cm":    np.array(Cm),
    }


if __name__ == "__main__":
    Re_wing = 18500
    output  = "data/wing_naca6409_polar.txt"

    run_xfoil_polar(
        airfoil     = "6409",
        Re          = Re_wing,
        alpha_start = -10,
        alpha_end   = 20,
        alpha_step  = 0.5,
        n_iter      = 300,    # high iteration count critical at low Re
        output_file = output
    )

    polar = parse_xfoil_polar(output)
    print(f"Alpha range: {polar['alpha'].min():.1f} to {polar['alpha'].max():.1f} deg")
    print(f"Max Cl/Cd:   {(polar['Cl']/polar['Cd']).max():.1f} at alpha = "
          f"{polar['alpha'][(polar['Cl']/polar['Cd']).argmax()]:.1f} deg")