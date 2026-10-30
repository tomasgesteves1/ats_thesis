#!/usr/bin/env python3
"""
Thesis Plot Style & Formatting Configuration
Cooperative Control of a Tethered UAV-USV System (MSc Thesis - IST)

Standardized styling, typography (LaTeX Computer Modern math font),
color palette, and dual vector/raster export (PDF and PNG) for all thesis figures.
"""

import os
import matplotlib as mpl
import matplotlib.pyplot as plt

# Universal Color Palette for Thesis Figures
# Rule 1: Neutral Gray (#666666) for simulation / ground truth baseline reference
# Rule 2: Deterministic color cycle for model predictions & comparison lines
# Rule 3: Solid lines for proposed/primary results; distinct styling for baselines
GROUND_TRUTH_COLOR = "#666666"
SIMULATION_COLOR = "#666666"
SECONDARY_COLOR = "#888888"

COLOR_CYCLE = [
    "#1f77b4",  # Series 1 / Proposed Model (Deep Blue)
    "#ff7f0e",  # Series 2 / Alternative 1  (Warm Orange)
    "#2ca02c",  # Series 3 / Alternative 2  (Forest Green)
    "#d62728",  # Series 4 / Alternative 3  (Crimson Red)
    "#9467bd",  # Series 5 / Alternative 4  (Purple)
    "#8c564b",  # Series 6 (Brown)
    "#e377c2",  # Series 7 (Pink)
    "#7f7f7f",  # Series 8 (Gray)
    "#bcbd22",  # Series 9 (Olive)
    "#17becf",  # Series 10 (Cyan)
]

# Visual styling rules for lines
# - Simulation / Ground Truth: Neutral gray solid line, slightly thicker baseline
# - Prediction Model (Proposed): Vivid color solid line (focal point)
SIMULATION_LINESTYLE = "-"
MODEL_LINESTYLE = "-"
SIMULATION_LINEWIDTH = 1.25
MODEL_LINEWIDTH = 1.0

# Legacy alias dictionary preserved for backwards-compatibility
THESIS_COLORS = {
    "ground_truth": GROUND_TRUTH_COLOR,
    "simulation": SIMULATION_COLOR,
    "reference": SECONDARY_COLOR,
    "model": COLOR_CYCLE[0],
    "slack_2": COLOR_CYCLE[0],
    "slack_5": COLOR_CYCLE[1],
    "slack_10": COLOR_CYCLE[2],
    "slack_15": COLOR_CYCLE[3],
    "uav": COLOR_CYCLE[0],
    "usv": COLOR_CYCLE[1],
    "target": COLOR_CYCLE[4],
    "model_fit": COLOR_CYCLE[0],
    "distance": "#333333",
    "limit": COLOR_CYCLE[3],
}


# Thesis page: A4 with 2.5 cm margins, text width of 455.24 pt (160 mm)
THESIS_TEXT_WIDTH = 455.24 / 72.27   # in
MM = 1 / 25.4                        # in

# Printed size of every thesis plot: (fraction of the text width, height in mm). The figures are drawn at this size
# and included in LaTeX at their natural size (no width=), so that the text is always 8 pt (labels) and 7 pt (ticks,
# legends) on paper. To resize a figure, change it here and run its script again.
# Width classes: 0.32 (three side by side), 0.48 (two side by side), 0.65 (alone), 1.0 (stacked / catalogue).
FIG_SIZES = {
    # 4.1 Simulation environment
    'slack_geometry': (0.48, 50), 'slack_tension': (0.48, 50),
    # 4.2 Model validation
    'usv_actuator_inputs_thrust': (0.48, 50), 'usv_actuator_inputs_azimuth': (0.48, 50),
    'usv_velocity_u_validation': (0.32, 40), 'usv_velocity_v_validation': (0.32, 40),
    'usv_velocity_r_validation': (0.32, 40),
    'usv_prediction_trajectory': (0.65, 75),
    'usv_prediction_error_pos': (0.48, 48), 'usv_prediction_error_psi': (0.48, 48),
    'usv_prediction_error_vel': (0.48, 48), 'usv_prediction_error_r': (0.48, 48),
    'uav_actuator_inputs_attitude': (0.32, 40), 'uav_actuator_inputs_yaw': (0.32, 40),
    'uav_actuator_inputs_thrust': (0.32, 40),
    'uav_attitude_roll_validation': (0.32, 40), 'uav_attitude_pitch_validation': (0.32, 40),
    'uav_attitude_yaw_validation': (0.32, 40),
    'uav_acceleration_x_validation': (0.32, 40), 'uav_acceleration_y_validation': (0.32, 40),
    'uav_acceleration_z_validation': (0.32, 40),
    'uav_prediction_trajectory': (0.65, 75),
    'uav_prediction_error_pos': (0.48, 48), 'uav_prediction_error_vel': (0.48, 48),
    'model_fit_comparison': (0.65, 48), 'model_fit_fx': (0.48, 52), 'model_fit_fz': (0.48, 52),
    'model_fit_residuals': (0.65, 48),
    # 4.3 Controller implementation
    'usv_solver_time': (0.48, 55), 'usv_solver_residual': (0.48, 55),
    'uav_solver_time': (0.48, 55), 'uav_solver_residual': (0.48, 55),
    # 4.4 Manoeuvres
    'maneuver_catalogue': (1.0, 115),
    # 5.1 Tracking
    'usv_track_top': (0.65, 50),
    'usv_track_thrust': (0.48, 50), 'usv_track_azimuth': (0.48, 50),
    'usv_track_error': (0.48, 50), 'usv_track_heading': (0.48, 50),
    'usv_limits_top': (0.65, 60), 'usv_limits_inputs': (1.0, 110),
    'crescent_top': (0.48, 45), 'crescent_track_top': (0.48, 45),
    'crescent_cmd_tilt': (0.48, 50), 'crescent_cmd_thrust': (0.48, 50),
    'crescent_error': (0.65, 52),
    'limits_top': (0.65, 65), 'limits_horizontal': (1.0, 80), 'limits_vertical': (1.0, 95),
    # 5.2 Missions
    'inspection_paths': (0.48, 62), 'inspection_timeline': (0.48, 78), 'inspection_error': (0.48, 62),
    'recovery_timeline': (0.65, 60),
    'patrol_paths': (0.48, 60), 'patrol_timeline': (0.48, 78),
    # 5.3 Disturbances
    'landing_nominal': (0.48, 65), 'waves_landing': (0.65, 80), 'wind_landing': (0.48, 65),
    'wind_offset': (0.48, 60), 'wind_force': (0.48, 60),
}


def fig_size(name, default=None):
    """Size (width, height) in inches of a thesis plot, from FIG_SIZES (or `default` for figures not in the thesis)."""
    if name not in FIG_SIZES:
        return default
    frac, h_mm = FIG_SIZES[name]
    return (frac * THESIS_TEXT_WIDTH, h_mm * MM)


def set_thesis_style():
    """
    Thesis plots: drawn at their printed size (FIG_SIZES) in Nimbus Sans, the Helvetica of the thesis text, with 8 pt
    labels and 7 pt ticks and legends, so that every figure has the same text size on paper.
    """
    plt.style.use("default")
    from matplotlib import font_manager
    for name in ("NimbusSans-Regular", "NimbusSans-Italic", "NimbusSans-Bold", "NimbusSans-BoldItalic"):
        path = os.path.join(NIMBUS_SANS, name + ".otf")
        if os.path.exists(path):
            font_manager.fontManager.addfont(path)

    mpl.rcParams.update({
        # Typography: the sans-serif of the thesis text, also for the mathematics
        "font.family": "sans-serif",
        "font.sans-serif": ["Nimbus Sans", "TeX Gyre Heros", "Liberation Sans", "DejaVu Sans"],
        "mathtext.fontset": "custom",
        "mathtext.rm": "Nimbus Sans",
        "mathtext.it": "Nimbus Sans:italic",
        "mathtext.bf": "Nimbus Sans:bold",
        "mathtext.sf": "Nimbus Sans",
        "text.usetex": False,

        # Font sizes on paper
        "font.size": 8.0,
        "axes.titlesize": 8.0,
        "axes.labelsize": 8.0,
        "xtick.labelsize": 7.0,
        "ytick.labelsize": 7.0,
        "legend.fontsize": 7.0,
        "figure.titlesize": 8.0,

        # Line widths and markers on paper
        "lines.linewidth": 1.0,
        "lines.markersize": 3.0,
        "axes.linewidth": 0.6,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "xtick.minor.width": 0.4,
        "ytick.minor.width": 0.4,
        "xtick.major.size": 2.5,
        "ytick.major.size": 2.5,
        "xtick.minor.size": 1.5,
        "ytick.minor.size": 1.5,

        # Grid Style (subtle, non-distracting)
        "axes.grid": True,
        "grid.alpha": 0.45,
        "grid.linestyle": "--",
        "grid.linewidth": 0.4,
        "grid.color": "#b0b0b0",

        # Axes Spines & Background
        "axes.edgecolor": "#333333",
        "axes.facecolor": "#ffffff",
        "figure.facecolor": "#ffffff",
        "axes.prop_cycle": mpl.cycler(color=COLOR_CYCLE),

        # Legend Box
        "legend.frameon": True,
        "legend.framealpha": 0.92,
        "legend.facecolor": "#ffffff",
        "legend.edgecolor": "#cccccc",
        "legend.borderpad": 0.3,
        "legend.labelspacing": 0.3,
        "legend.handlelength": 1.6,

        # Export & Font Embedding
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
        "pdf.fonttype": 3,    # OpenType (CFF) outlines of Nimbus Sans are embedded as Type 3
        "ps.fonttype": 3,
    })


NIMBUS_SANS = "/usr/share/fonts/opentype/urw-base35"


def legend_above(ax, ncol=None, **kwargs):
    """Legend in one row above the axes, without a frame: for the narrow panels (three side by side), where a legend
    inside the axes hides the curves."""
    handles, labels = ax.get_legend_handles_labels()
    if ncol is None:
        ncol = len(labels) if sum(len(lab) for lab in labels) <= 30 else 1   # one row only if it fits the panel
    return ax.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=ncol,
                     frameon=False, borderaxespad=0.2, handlelength=1.4, columnspacing=1.0, **kwargs)


PROPER_NAMES = {"Gazebo", "Foxglove", "Harmonic", "Torqeedo", "Holybro", "Earth"}


def normalise_labels(fig):
    """Axis labels in the thesis convention: units in parentheses, "Quantity (unit)", and an initial capital letter
    (except single-letter coordinates such as "x (m)" and labels that start with mathematics)."""
    import re
    for ax in fig.get_axes():
        for get, set_ in ((ax.get_xlabel, ax.set_xlabel), (ax.get_ylabel, ax.set_ylabel)):
            t = get()
            if not t:
                continue
            t2 = re.sub(r"\[([^\[\]]*)\]\s*$", r"(\1)", t)
            first = t2.split(" ")[0]
            if first[:1].islower() and len(first) > 1 and not first.startswith("$"):
                t2 = t2[0].upper() + t2[1:]
            if t2 != t:
                set_(t2)
    # legend entries start with a capital letter too ("Applied", "Gain 0"), except mathematics and symbols
    legends = [ax.get_legend() for ax in fig.get_axes()] + list(fig.legends)
    for leg in legends:
        if leg is None:
            continue
        for txt in leg.get_texts():
            t = txt.get_text()
            first = t.split(" ")[0] if t else ""
            if first[:1].islower() and len(first) > 1 and not first.startswith("$"):
                t = t[0].upper() + t[1:]
            # sentence case: "Prediction Model" -> "Prediction model", keeping acronyms (USV, PX4), names with inner
            # capitals (MoorDyn) and proper names
            words = t.split(" ")
            for i in range(1, len(words)):
                w = words[i]
                if re.fullmatch(r"[A-Z][a-z]+[,.:;]?", w) and w.strip(",.:;") not in PROPER_NAMES:
                    words[i] = w[0].lower() + w[1:]
            t = " ".join(words)
            if t != txt.get_text():
                txt.set_text(t)


# Extended abstract (IEEEtran, journal, A4): columns of 252 pt (88.6 mm) and text of 516 pt (181.4 mm)
EA_COLUMN_WIDTH = 252.0 / 72.27   # in
EA_TEXT_WIDTH = 516.0 / 72.27     # in


def set_ea_style():
    """Figures of the extended abstract: the thesis style (Nimbus Sans, 8/7 pt), drawn at their printed size (column or
    text width of the IEEE template), so that they are included without scaling, like the figures of the thesis."""
    set_thesis_style()
    mpl.rcParams.update({
        "savefig.bbox": "standard",   # the figure keeps the size it was drawn at
        "savefig.pad_inches": 0.0,
    })


def save_figure(fig, output_path, save_png=False, dpi=300):
    """
    Saves the figure in vector (.pdf) for LaTeX inclusion.
    Optionally saves raster (.png) if save_png=True.
    
    Args:
        fig: matplotlib.figure.Figure instance
        output_path: Path with or without extension
        save_png: Whether to also save PNG raster (default: False)
        dpi: Resolution for raster export (default: 300)
    """
    base, _ = os.path.splitext(output_path)
    out_dir = os.path.dirname(os.path.abspath(base))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    normalise_labels(fig)
    pdf_path = f"{base}.pdf"
    fig.savefig(pdf_path, format="pdf")
    print(f"Saved vector figure (LaTeX ready): {pdf_path}")

    # Never pollute LaTeX directories with PNG files (thesis uses vector PDFs exclusively)
    is_latex_dir = "latex" in os.path.abspath(base).split(os.sep)
    if save_png and not is_latex_dir:
        png_path = f"{base}.png"
        fig.savefig(png_path, format="png", dpi=dpi, bbox_inches="tight")
        print(f"Saved raster figure (300 DPI):     {png_path}")
