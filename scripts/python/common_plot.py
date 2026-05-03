from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

FIGURES_DIR = Path("figures")
PROCESSED_DIR = Path("data/processed")

rcParams.update({
    "font.family": "serif",
    "font.size": 6,
    "axes.labelsize": 6,
    "axes.titlesize": 6,
    "xtick.labelsize": 6,
    "ytick.labelsize": 6,
    "legend.fontsize": 6,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

def ieee_axes(ax):
    ax.tick_params(direction="in", which="both", labelsize=6)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", linestyle=":", linewidth=0.4, alpha=0.9)

def ensure_dirs():
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    PROCESSED_DIR.mkdir(parents=True, exist_ok=True)

def load_df(path):
    df = pd.read_csv(path)
    if "per_exchange_fail" in df.columns:
        df["S_ex"] = 1.0 - df["per_exchange_fail"]
    elif "S_ex" not in df.columns:
        raise ValueError("Missing required column: per_exchange_fail or S_ex")

    if "pacing_mode" in df.columns:
        df["pacing"] = df["pacing_mode"].replace({
            "baseline": "Baseline",
            "stochastic": "Stochastic",
            "radioaware_phase_bounded": "Phase-bounded",
            "fixed": "Deterministic",
        })
    elif "pacing" not in df.columns:
        raise ValueError("Missing required column: pacing_mode or pacing")

    return df

def format_lambda_value(lam: float) -> str:
    if float(lam).is_integer():
        return str(int(lam))
    return f"{lam:g}"

def lambda_axis_label() -> str:
    return r"Offered load $\lambda$ (exchanges/s)"

def lambda_panel_title(lam: float) -> str:
    return rf"$\lambda = {format_lambda_value(lam)}$ exchanges/s"
