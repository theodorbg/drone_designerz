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
            # Skip comments and header lines
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

    # Convert all lists to numpy arrays
    return {k: np.array(v) for k, v in data.items()}


# ── Usage ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    polar_us = read_polar_txt("clf5605_us_fp_polar.txt")
    polar_jp = read_polar_txt("clf5605_jp_f_polar.txt")

    print("clf5605-us-fp:")
    for k, v in polar_us.items():
        print(f"  {k}: {v}")

    print("\nclf5605-jp-f:")
    for k, v in polar_jp.items():
        print(f"  {k}: {v}")
