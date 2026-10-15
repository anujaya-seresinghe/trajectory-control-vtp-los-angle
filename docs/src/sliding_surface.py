"""Sliding surface s = x1 + (1/beta) * sign(x2) |x2|^alpha = 0 of the terminal non-singular sliding mode controller,
in the phase plane x1 = lambda - lambda_d, x2 = lambda_dot - lambda_d_dot. One panel per alpha, one curve per beta.

    python3 docs/src/sliding_surface.py   ->  docs/img/sliding_surface.png
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ALPHAS = [1.02, 1.941]             # TRAJ_SMC_ALPHA, 1 < alpha < 2
BETAS = [0.1, 0.7]                 # TRAJ_SMC_BETA
X1_MAX = 0.5                       # [rad]

# Colours of the other figures in docs/img
BG = "#131313"
FG = "#e8e8e8"
GRID = "#2c2c2c"
FRAME = "#33cc33"
COLORS = ["#1ec8c8", "#c040c0", "#a8d8ff", "#e6ff99"]  # most contrasting first

OUT = Path(__file__).resolve().parents[1] / "img" / "sliding_surface.png"

plt.rcParams.update({
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG,
    "text.color": FG, "axes.labelcolor": FG, "xtick.color": FG, "ytick.color": FG,
    "axes.edgecolor": "#5a5a5a", "font.size": 11, "mathtext.fontset": "dejavusans",
})

fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True)
x1 = np.linspace(-X1_MAX, X1_MAX, 801)

for ax, alpha in zip(axes, ALPHAS):
    for beta, color in zip(BETAS, COLORS):
        # s = 0  <=>  x2 = -sign(x1) (beta |x1|)^(1/alpha)
        x2 = -np.sign(x1) * (beta * np.abs(x1)) ** (1.0 / alpha)
        label = rf"$\beta$ = {beta:g}"
        ax.plot(x1, x2, color=color, linewidth=2.2, label=label)

    ax.axhline(0, color="#5a5a5a", linewidth=0.8)
    ax.axvline(0, color="#5a5a5a", linewidth=0.8)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_xlim(-X1_MAX, X1_MAX)
    ax.set_title(rf"$\alpha$ = {alpha:g}", fontsize=14, pad=10)
    ax.set_xlabel(r"$x_1 = \lambda - \lambda_d$  [rad]")
    legend = ax.legend(loc="upper right", frameon=True, fontsize=10)
    legend.get_frame().set_facecolor(BG)
    legend.get_frame().set_edgecolor(FRAME)
    legend.get_frame().set_linewidth(1.5)

axes[0].set_ylabel(r"$x_2 = \dot\lambda - \dot\lambda_d$  [rad/s]")
fig.suptitle(r"Sliding surface  $s = x_1 + \frac{1}{\beta}\,x_2^{\,\alpha} = 0$", fontsize=15, y=0.98)
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig(OUT, dpi=150)
print(OUT)
