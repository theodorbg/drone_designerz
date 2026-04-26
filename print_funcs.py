import numpy as np

def print_q2_comparison_latex(best_dual, best_quad, caption="Q2 design comparison", label="tab:q2_comparison"):
    """Print Q2 comparison table in LaTeX format."""
    rows = [
        ("Rotor radius [m]", f"{best_dual.rotor_radius:.2f}", f"{best_quad.rotor_radius:.2f}"),
        ("Number of blades [-]", f"{int(best_dual.N_blades)}", f"{int(best_quad.N_blades)}"),
        ("Total hover power [W]", f"{best_dual.hover_power:.0f}", f"{best_quad.hover_power:.0f}"),
        ("Total mass with payload [kg]", f"{best_dual.mass:.2f}", f"{best_quad.mass:.2f}"),
        ("Hover flight time [s]", f"{best_dual.total_hover_time:.0f}", f"{best_quad.total_hover_time:.0f}"),
    ]

    print("\n% ---------- LaTeX table: Q2 comparison ----------")
    print("\\begin{table}[h!]")
    print("  \\centering")
    print(f"  \\caption{{{caption}}}")
    print(f"  \\label{{{label}}}")
    print("  \\begin{tabular}{lcc}")
    print("    \\hline")
    print("    Metric & Dual Copter & Quad Copter \\\\")
    print("    \\hline")
    for metric, dual_val, quad_val in rows:
        print(f"    {metric} & {dual_val} & {quad_val} \\\\")
    print("    \\hline")
    print("  \\end{tabular}")
    print("\\end{table}")
    print("% -----------------------------------------------\n")


def print_q3_optimums(results, N_batt_max_2kg):
    """Print unconstrained and 2 kg-constrained optimum battery counts and times."""
    for name in ["Dual Copter", "Quad Copter"]:
        x = np.array(results[name]["N_batt"], dtype=int)
        y = np.array(results[name]["time_min"], dtype=float)

        if len(x) == 0:
            print(f"\n{name}: no converged points.")
            continue

        i_opt = int(np.argmax(y))
        N_opt = int(x[i_opt])
        t_opt = float(y[i_opt])

        mask_2kg = x <= N_batt_max_2kg
        if not np.any(mask_2kg):
            print(f"\n{name}: no converged points within 2 kg constraint.")
            continue

        xc = x[mask_2kg]
        yc = y[mask_2kg]
        i_c = int(np.argmax(yc))
        N_2kg = int(xc[i_c])
        t_2kg = float(yc[i_c])

        print(f"\n{name}")
        print(f"  Unconstrained optimum: N_batteries = {N_opt}, flight time = {t_opt:.2f} min")
        if N_opt > N_batt_max_2kg:
            print(f"  Above 2 kg limit -> best within 2 kg: N_batteries = {N_2kg}, flight time = {t_2kg:.2f} min")
        else:
            print("  Within 2 kg limit -> constrained optimum is the same.")

def _latex_sci(x, sigfigs=2):
    if x == 0:
        return "0"
    exp = int(np.floor(np.log10(abs(x))))
    mant = x / (10 ** exp)
    return f"{mant:.{sigfigs}f} \\times 10^{{{exp}}}"

def print_q4_reynolds_latex(reynolds_results, reynolds_diffs):
    """
    Print Reynolds numbers for Earth/Mars and the difference as a LaTeX table.
    Expected keys:
      reynolds_results[("Earth", "Dual Copter")]
      reynolds_results[("Mars", "Dual Copter")]
      reynolds_results[("Earth", "Quad Copter")]
      reynolds_results[("Mars", "Quad Copter")]
    """
    dual_re_earth = reynolds_results[("Earth", "Dual Copter")]
    dual_re_mars = reynolds_results[("Mars", "Dual Copter")]
    quad_re_earth = reynolds_results[("Earth", "Quad Copter")]
    quad_re_mars = reynolds_results[("Mars", "Quad Copter")]

    dual_diff = reynolds_diffs["Dual Copter"]
    quad_diff = reynolds_diffs["Quad Copter"]
    print("% ---------- LaTeX table: Q4 Reynolds numbers ----------")
    print("\\begin{table}[h]")
    print("\\centering")
    print("\\begin{tabular}{lccc}")
    print("\\hline")
    print("           & $Re_{earth}$ & $Re_{mars}$ & $\\Delta Re$ \\\\")
    print("\\hline")
    print(
        f"Two Rotor  & ${_latex_sci(dual_re_earth)}$ & ${_latex_sci(dual_re_mars)}$ & ${_latex_sci(dual_diff)}$ \\\\"
    )
    print(
        f"Four Rotor & ${_latex_sci(quad_re_earth)}$ & ${_latex_sci(quad_re_mars)}$ & ${_latex_sci(quad_diff)}$ \\\\"
    )
    print("\\hline")
    print("\\end{tabular}")
    print("\\end{table}")
    print("% -----------------------------------------------\n")
