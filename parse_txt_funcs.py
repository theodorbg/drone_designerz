import numpy as np

def read_polar_txt(filepath):
    """
    Read a polar text file produced by various tools (or return a DataFrame if passed).
    Robustly skips header/separator lines (e.g. '------') and any non-numeric rows.
    Returns a pandas.DataFrame with columns ['alpha','Cl','Cd','Cm', ...] when possible,
    otherwise a dict of lists.
    """
    # If None is passed, return None (allows optional polar data)
    if filepath is None:
        return None

    try:
        import pandas as pd
    except Exception:
        pd = None

    # If caller already passed a DataFrame, just return it
    if pd is not None and isinstance(filepath, pd.DataFrame):
        return filepath

    data = {"alpha": [], "Cl": [], "Cd": [], "Cm": []}

    # Accept file-like objects too
    if not isinstance(filepath, (str, bytes, os.PathLike)):
        f = filepath
        close_after = False
    else:
        f = open(filepath, "r")
        close_after = True

    try:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # Skip obvious separators or header lines
            if set(line) <= set("-= ") or line.lower().startswith(("alpha", "a ", "acl")):
                continue
            parts = line.split()
            # need at least alpha, Cl, Cd (Cm optional)
            if len(parts) < 3:
                continue
            # try to parse numeric values; skip the line if any conversion fails
            try:
                a = float(parts[0])
                cl = float(parts[1])
                cd = float(parts[2])
                cm = float(parts[3]) if len(parts) > 3 else 0.0
            except ValueError:
                # line contains non-numeric tokens (e.g. '------'), skip it
                continue
            data["alpha"].append(a)
            data["Cl"].append(cl)
            data["Cd"].append(cd)
            data["Cm"].append(cm)
    finally:
        if close_after:
            f.close()

    if pd is not None:
        try:
            return pd.DataFrame(data)
        except Exception:
            return data
    return data


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

import pandas as pd

def xfoil_polar_txt_to_dataframe(filepath: str) -> pd.DataFrame:
    rows = []
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    data_started = False

    for line in lines:
        s = line.strip()

        if not data_started:
            if s.lower().startswith("alpha") and "cl" in s.lower():
                data_started = True
            continue

        if not s or s.startswith("-"):
            continue

        parts = s.split()
        if len(parts) < 7:
            continue

        try:
            rows.append({
                "alpha": float(parts[0]),
                "Cl": float(parts[1]),
                "Cd": float(parts[2]),
                "CDp": float(parts[3]),
                "CM": float(parts[4]),
                "Top_Xtr": float(parts[5]),
                "Bot_Xtr": float(parts[6]),
            })
        except ValueError:
            continue

    return pd.DataFrame(rows)


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