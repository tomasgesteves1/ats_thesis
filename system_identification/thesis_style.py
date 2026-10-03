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

# Consistent Color Palette for Figures
THESIS_COLORS = {
    # Slack Factor Experiments
    "slack_2": "#1f77b4",    # 2% Slack (factor 1.02) - Deep Blue
    "slack_5": "#ff7f0e",    # 5% Slack (factor 1.05) - Warm Orange
    "slack_10": "#2ca02c",   # 10% Slack (factor 1.10) - Forest Green
    "slack_15": "#d62728",   # 15% Slack (factor 1.15) - Crimson Red
    
    # Kinematics & Tracking
    "ground_truth": "#111111", # Ground Truth / Measured - Solid Black
    "reference": "#7f7f7f",    # Reference Path / Setpoint - Gray
    "model_fit": "#d62728",    # Model Prediction / Identification - Red
    "distance": "#333333",     # Geometric Distance (d) - Charcoal
    
    # Multi-Agent Coordination
    "uav": "#1f77b4",          # UAV (Aerial)
    "usv": "#2ca02c",          # USV (Marine Vessel)
    "target": "#9467bd",       # Target Vessel (Pursuit)
    
    # Constraints & Thresholds
    "limit": "#d62728",        # Constraint Bounds / Limits - Red
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

    if save_png:
        png_path = f"{base}.png"
        fig.savefig(png_path, format="png", dpi=dpi, bbox_inches="tight")
        print(f"Saved raster figure (300 DPI):     {png_path}")
