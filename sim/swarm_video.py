"""Video of a four-drone SITL swarm mission with an injected fault (truth positions from swarm_report.fly_mission).

    make video     # -> docs/img/swarm-sitl.gif (README) and build/swarm-sitl.mp4 (needs ffmpeg)
"""
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter, FuncAnimation  # noqa: E402

from failsafe_report import NAMES  # noqa: E402
from swarm_report import CHUNK, HALF, fly_mission  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
GIF, MP4 = ROOT / "docs" / "img" / "swarm-sitl.gif", ROOT / "build" / "swarm-sitl.mp4"
FAULT_DRONE = 3
FPS, SPEED, TRAIL_S, ARM = 25, 2.0, 3.0, 0.09  # playback: 2x real time; arm length drawn larger than the 2-inch frame
BG, INK, MUTED = "#0f1216", "#e8e6e1", "#8a8f98"
COLORS = ["#29b6f6", "#76b041", "#f3a712", "#e4572e"]


def main() -> None:
    run = fly_mission(fault="command", fault_drone=FAULT_DRONE, fault_phase="swap")
    assert run["passed"], "swarm scenario failed; fix that before filming it"
    swarm = run["swarm"]
    pos = np.array(swarm.pos)  # (chunks, drones, xyz), one row per CHUNK
    safety = swarm.drones[FAULT_DRONE].log.arrays()["safety"].astype(int)[-len(pos):]
    frames = range(0, len(pos), round(SPEED / FPS / CHUNK))

    fig = plt.figure(figsize=(8, 6), facecolor=BG)
    ax = fig.add_subplot(projection="3d", facecolor=BG)
    fig.subplots_adjust(0, 0, 1, 0.94)
    ax.set(xlim=(-1, 1), ylim=(-1, 1), zlim=(0, 2.3), xlabel="x, m", ylabel="y, m", zlabel="height, m")
    ax.set_box_aspect((2, 2, 2.3))
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color(BG)
        axis.label.set_color(MUTED)
        axis._axinfo["grid"]["color"] = "#252a31"
    ax.tick_params(colors=MUTED, labelsize=8)
    square = HALF * np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]])
    ax.plot(square[:, 0], square[:, 1], 0, color="#3a4049", linewidth=1, linestyle=(0, (4, 3)))
    fig.text(0.03, 0.95, "ESP-FC Next: four-drone swarm, software in the loop", color=INK, fontsize=13, weight="bold")
    status = fig.text(0.03, 0.905, "", color=MUTED, fontsize=10)

    trails = [ax.plot([], [], [], color=c, linewidth=1.2, alpha=0.55)[0] for c in COLORS]
    shadows = [ax.plot([], [], [], "o", color=c, alpha=0.25, markersize=5)[0] for c in COLORS]
    arms = [[ax.plot([], [], [], color=c, linewidth=2.5, solid_capstyle="round")[0] for _ in range(2)] for c in COLORS]

    def draw(k: int):
        ax.view_init(elev=22, azim=-60 + 40 * k / len(pos))
        tail = max(0, k - round(TRAIL_S / CHUNK))
        for i in range(len(COLORS)):
            x, y, z = pos[k, i]
            trails[i].set_data_3d(*pos[tail:k + 1, i].T)
            shadows[i].set_data_3d([x], [y], [0])
            for arm, s in zip(arms[i], (1, -1)):
                arm.set_data_3d([x - ARM, x + ARM], [y - s * ARM, y + s * ARM], [z, z])
        state = NAMES[safety[k]]
        fault = "" if state == "NORMAL" else f"   drone {FAULT_DRONE}: radio link lost, failsafe {state}"
        status.set_text(f"t = {k * CHUNK:4.1f} s   real C flight firmware x4, 6-DOF model{fault}")
        status.set_color(MUTED if not fault else COLORS[FAULT_DRONE])

    anim = FuncAnimation(fig, draw, frames=frames)
    MP4.parent.mkdir(parents=True, exist_ok=True)
    anim.save(MP4, writer=FFMpegWriter(fps=FPS, bitrate=1800), dpi=120, savefig_kwargs={"facecolor": BG})
    # GitHub READMEs autoplay GIFs but not repository MP4s: same footage, half frame rate and size.
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", MP4, "-filter_complex",
                    "fps=10,scale=520:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];"
                    "[b][p]paletteuse=dither=none", GIF], check=True)
    print(MP4, GIF)


if __name__ == "__main__":
    main()
