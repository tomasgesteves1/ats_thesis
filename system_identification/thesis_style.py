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


def set_thesis_style():
    """
    Applies publication-quality Matplotlib parameters aligned with LaTeX
    standards for MSc Thesis at Instituto Superior Técnico.
    """
    plt.style.use("default")
    
    mpl.rcParams.update({
        # Typography & Math Rendering (LaTeX Computer Modern style)
        "font.family": "serif",
        "font.serif": ["Computer Modern Roman", "DejaVu Serif", "Times New Roman", "serif"],
        "mathtext.fontset": "cm",
        "mathtext.rm": "serif",
        "text.usetex": False,  # Pure portable mathtext, no external TeX engine crash risk

        # Font Sizes (optimized for textwidth ~6.0-6.5 inches)
        "font.size": 10.0,
        "axes.titlesize": 11.0,
        "axes.labelsize": 10.0,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 8.5,
        "figure.titlesize": 12.0,

        # Line Widths & Markers
        "lines.linewidth": 1.2,
        "lines.markersize": 3.5,

        # Grid Style (subtle, non-distracting)
        "axes.grid": True,
        "grid.alpha": 0.45,
        "grid.linestyle": "--",
        "grid.linewidth": 0.5,
        "grid.color": "#b0b0b0",

        # Axes Spines & Background
        "axes.edgecolor": "#333333",
        "axes.linewidth": 0.8,
        "axes.facecolor": "#ffffff",
        "figure.facecolor": "#ffffff",

        # Legend Box
        "legend.frameon": True,
        "legend.framealpha": 0.92,
        "legend.facecolor": "#ffffff",
        "legend.edgecolor": "#cccccc",
        "legend.borderpad": 0.35,
        "legend.labelspacing": 0.3,
        "legend.handlelength": 1.8,

        # Export & Font Embedding
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.04,
        "pdf.fonttype": 42,   # Embed TrueType in PDF for vector quality without type 3 fonts
        "ps.fonttype": 42,
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

    pdf_path = f"{base}.pdf"
    fig.savefig(pdf_path, format="pdf", bbox_inches="tight")
    print(f"Saved vector figure (LaTeX ready): {pdf_path}")

    # Never pollute LaTeX directories with PNG files (thesis uses vector PDFs exclusively)
    is_latex_dir = "latex" in os.path.abspath(base).split(os.sep)
    if save_png and not is_latex_dir:
        png_path = f"{base}.png"
        fig.savefig(png_path, format="png", dpi=dpi, bbox_inches="tight")
        print(f"Saved raster figure (300 DPI):     {png_path}")
