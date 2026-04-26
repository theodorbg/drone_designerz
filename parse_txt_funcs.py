import numpy as np

def read_polar_txt(filepath):
    """
    Reads a CLF5605 experimental polar .txt file.
    Returns a dict with keys: alpha, Re, M, Cl, Cl_std, Cd, Cd_std
    Each key maps to a numpy array.
    """
    data = {
        "alpha":  [],
        "Re":     [],
        "M":      [],
        "Cl":     [],
        "Cl_std": [],
        "Cd":     [],
        "Cd_std": [],
    }

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#") or line == "" or line.startswith("alpha"):
                continue

            parts = line.split()
            if len(parts) == 7:
                data["alpha"].append(float(parts[0]))
                data["Re"].append(float(parts[1]))
                data["M"].append(float(parts[2]))
                data["Cl"].append(float(parts[3]))
                data["Cl_std"].append(float(parts[4]))
                data["Cd"].append(float(parts[5]))
                data["Cd_std"].append(float(parts[6]))

    return {k: np.array(v) for k, v in data.items()}

def read_blade_geometry_nasa(filepath):
    data = {
        "y": [],
        "c_R": [],
        "twist_deg": [],
        "radius": [],
        "chord": [],
        "thickness": [],
        "t_c": [],
        "airfoil": [],
    }

    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()

            # Skip non-data rows (e.g., header: y c_R twist_deg ...)
            try:
                y = float(parts[0])
            except (ValueError, IndexError):
                continue

            data["y"].append(y)
            data["c_R"].append(float(parts[1]))
            data["twist_deg"].append(float(parts[2]))
            data["radius"].append(float(parts[3]))
            data["chord"].append(float(parts[4]))
            data["thickness"].append(float(parts[5]))
            data["t_c"].append(float(parts[6]))
            data["airfoil"].append(parts[7])

    return {k: np.array(v) if k != "airfoil" else v for k, v in data.items()}


def load_rotor_parameters(filepath):
    params = {}

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#") or line == "" or line.startswith("parameter"):
                continue

            parts = line.split()
            params[parts[0]] = float(parts[1])

    return params