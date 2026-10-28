"""PNG of the power_sim transient: 3.5 V pack, 50 mOhm, 16 A motor step. Run: uv run --with matplotlib plot_power.py"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import power_sim  # noqa: E402

INK, MUTED, GRID, SURFACE = "#1f1f1e", "#6b6a64", "#e4e3de", "#fcfcfb"
rows = power_sim.trace()
t = [r[0] * 1000 for r in rows]
fig, (v, i) = plt.subplots(2, 1, sharex=True, figsize=(9, 5.5), height_ratios=(3, 2), facecolor=SURFACE)
for ax in (v, i):
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID, linewidth=0.8)
    ax.tick_params(colors=MUTED, labelcolor=INK)
    for s in ax.spines.values():
        s.set_visible(False)
v.plot(t, [r[1] for r in rows], color="#2a78d6", linewidth=2)
v.plot(t, [r[2] for r in rows], color="#eb6834", linewidth=2)
v.axhline(power_sim.VIN_FULL_LOAD, color=MUTED, linewidth=1, linestyle=(0, (4, 3)))
v.text(3, power_sim.VIN_FULL_LOAD - 0.02, "TPS63802 min input at 2 A: 2.3 V", color=MUTED, va="top")
v.set_ylim(2.15, 3.5)
low = min(rows, key=lambda r: r[2])
v.annotate(f"min {low[2]:.2f} V", (low[0] * 1000, low[2]), (low[0] * 1000 + 4, low[2] + 0.35), color=INK,
           arrowprops=dict(arrowstyle="-", color=MUTED))
v.set_ylabel("volts", color=INK)
v.legend(["VBAT (cell)", "VSYS (3.3 V converter input)"], frameon=False, loc="center right", labelcolor=INK)
i.plot(t, [r[3] for r in rows], color="#2a78d6", linewidth=2)
i.set_ylabel("battery current, A", color=INK)
i.set_xlabel("time, ms", color=INK)
v.set_title("1S power path: 16 A motor step, 3.5 V pack with 50 mOhm internal resistance (ngspice)",
            color=INK, loc="left", fontsize=11)
fig.tight_layout()
fig.savefig("out/power_transient.png", dpi=150)
print("out/power_transient.png")
