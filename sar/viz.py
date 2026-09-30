"""지도/경로/상태 시각화 (matplotlib, 없으면 조용히 비활성).

- save(path): 현재 상황 PNG 저장 (headless 안전, Agg 백엔드)
- 오프라인 sim과 Webots 컨트롤러 양쪽에서 사용
"""
import math

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:  # matplotlib 미설치 환경
    HAVE_MPL = False


class MapViz:
    def __init__(self, grid):
        self.grid = grid
        self.enabled = HAVE_MPL

    def save(self, path, pose=None, info=None, true_pose=None, world_extras=None):
        if not self.enabled:
            return
        g = self.grid
        fig, ax = plt.subplots(figsize=(7, 7), dpi=90)
        # 지도: unknown 회색, free 흰색, occupied 검정
        img = 0.5 - 0.5 * g.free_mask() + 0.5 * g.occupied_mask()
        extent = (g.origin_x, g.origin_x + g.n * g.res,
                  g.origin_y, g.origin_y + g.n * g.res)
        ax.imshow(1 - img, cmap="gray", origin="lower", extent=extent,
                  vmin=0, vmax=1)
        if info:
            wps = info.get("waypoints") or []
            if wps:
                ax.plot([p[0] for p in wps], [p[1] for p in wps],
                        "-", color="tab:blue", lw=1.5, label="path")
            crumbs = info.get("crumbs") or []
            if crumbs:
                ax.plot([p[0] for p in crumbs], [p[1] for p in crumbs],
                        ".", color="tab:orange", ms=2, label="crumbs")
            if info.get("goal"):
                ax.plot(*info["goal"], "x", color="tab:blue", ms=10)
            if info.get("target_est"):
                ax.plot(*info["target_est"], "*", color="red", ms=14,
                        label="target est")
        if pose is not None:
            ax.plot(pose[0], pose[1], "o", color="tab:green", ms=8)
            ax.arrow(pose[0], pose[1],
                     0.25 * math.cos(pose[2]), 0.25 * math.sin(pose[2]),
                     head_width=0.08, color="tab:green")
        if true_pose is not None:
            ax.plot(true_pose[0], true_pose[1], "+", color="purple", ms=10,
                    label="true pose")
        if world_extras:
            for (x, y, marker, color, label) in world_extras:
                ax.plot(x, y, marker, color=color, ms=10, label=label)
        state = info.get("state") if info else ""
        ax.set_title(f"{state}")
        ax.legend(loc="upper right", fontsize=7)
        ax.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(path)
        plt.close(fig)
