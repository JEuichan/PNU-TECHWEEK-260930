"""Webots 메인 컨트롤러 — sar 패키지의 미션 로직을 실로봇 루프에 연결.

실행 방법 (둘 중 하나):
1) Webots 월드에서 robot controller = "sar_controller" 로 두고 실행
   (Webots 환경설정의 Python command를 프로젝트 .venv 파이썬으로 지정)
2) extern 컨트롤러: 월드에서 controller = "<extern>" 으로 두고
   $ /Applications/Webots.app/Contents/MacOS/webots-controller \
       --stdout-redirect webots/controllers/sar_controller/sar_controller.py

당일 어댑테이션 포인트는 전부 sar/config.py 와 robot_io.py 에 있다.
"""
import math
import os
import sys

# 저장소 루트를 import 경로에 추가 — 'sar' 패키지가 보일 때까지 상위로
# 탐색 (webots/controllers/... 배치와 저장소 루트 직속 controllers/...
# 배치 모두 지원)
_ROOT = os.path.dirname(os.path.abspath(__file__))
for _ in range(6):
    if os.path.isdir(os.path.join(_ROOT, "sar")):
        break
    _ROOT = os.path.dirname(_ROOT)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sar.config import default_config           # noqa: E402
from sar.mapping import OccupancyGrid           # noqa: E402
from sar.odometry import PoseEstimator          # noqa: E402
from sar.state_machine import Mission           # noqa: E402
from sar.viz import MapViz                      # noqa: E402
from robot_io import RobotIO                    # noqa: E402

SNAPSHOT_EVERY_S = 5.0        # 지도 PNG 저장 주기 (0이면 끔)
SNAPSHOT_DIR = os.path.join(_ROOT, "out")
# YOLO 가구 스캔: 탁자·의자류를 미리 인식해 밑으로 파고드는 경로를 예방
FURNITURE_CLASSES = {"dining table", "chair", "couch", "bench", "bed"}
FURNITURE_SCAN_S = 3.0


def main():
    cfg = default_config()
    override = os.path.join(_ROOT, "config_override.json")
    if os.path.exists(override):
        cfg.apply_overrides(override)
        print(f"[sar] config override 적용: {override}")

    io = RobotIO(cfg)
    dt = io.timestep / 1000.0
    start = (cfg.mission.start_x, cfg.mission.start_y, cfg.mission.start_theta)
    grid = OccupancyGrid(cfg, center_xy=start[:2])
    estimator = PoseEstimator(cfg, start)
    mission = Mission(cfg, grid)
    viz = MapViz(grid)
    if SNAPSHOT_EVERY_S > 0:
        os.makedirs(SNAPSHOT_DIR, exist_ok=True)

    print(f"[sar] timestep={io.timestep}ms lidar={io.lidar is not None} "
          f"camera={io.camera is not None}")
    mission.detector.yolo_warmup()

    now = 0.0
    step_i = 0
    last_snap = -1e9
    last_cam = -1e9
    last_yolo = -1e9
    yolo_cache = []
    last_state = None
    done_logged = False
    found_saved = False
    last_visited = 0
    last_furn = -1e9
    while io.step() != -1:
        now += dt
        step_i += 1
        left, right = io.wheel_positions()
        pose = estimator.update(left, right, compass_raw=io.compass_raw())
        angles, ranges = io.lidar_scan()
        if angles is None:
            io.drive(0.0, 0.0)
            continue
        # 지도 갱신은 Mission.step 내부에서 (동적 빔 제외 후) 수행
        img = io.camera_image()
        v, w, info = mission.step(now, pose, angles, ranges, img)
        io.drive(v, w)

        # YOLO 가구 스캔 — 탁자류 방향의 LiDAR 최소거리로 위치를 잡아
        # '밑으로 파고들지 않을 구역'으로 등록 (갇힘의 예방 레이어)
        if now - last_furn >= FURNITURE_SCAN_S and img is not None \
                and getattr(mission.detector, "_yolo", None) is not None:
            last_furn = now
            try:
                import numpy as _np
                from sar.geometry import wrap_angle as _wrap
                h_, w_ = img.shape[:2]
                f_px = (w_ / 2) / math.tan(cfg.camera.hfov / 2)
                a_arr = _np.asarray(angles)
                r_arr = _np.asarray(ranges)
                for name, conf, (x0, y0, x1, y1) in \
                        mission.detector.yolo_boxes(img):
                    if name not in FURNITURE_CLASSES or conf < 0.35:
                        continue
                    if y1 < h_ * 0.5:     # 화면 위쪽 절반뿐 → 원거리/벽면
                        continue
                    cx_b = (x0 + x1) / 2.0
                    b = math.atan2(w_ / 2.0 - cx_b, f_px)
                    sel = _np.abs(_wrap(a_arr - b)) < 0.2
                    rr2 = r_arr[sel]
                    rr2 = rr2[_np.isfinite(rr2) & (rr2 > 0.15)]
                    if rr2.size == 0:
                        continue
                    d = float(rr2.min())
                    if d > 2.5:
                        continue
                    fx = pose[0] + d * math.cos(pose[2] + b)
                    fy = pose[1] + d * math.sin(pose[2] + b)
                    mission.note_furniture((fx, fy))
            except Exception as e:
                print(f"[sar] 가구 스캔 실패: {e}")

        if info["state"] != last_state:
            last_state = info["state"]
            print(f"[sar] t={now:6.1f}s → {last_state} "
                  f"pose=({pose[0]:+.2f},{pose[1]:+.2f})")
        # 목표 확정 순간의 카메라 프레임 저장 — "무엇을 목표로 봤는가"
        # 검증용 (디코이 오인 디버깅에 결정적)
        if info["found"] and not found_saved and img is not None:
            found_saved = True
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                os.makedirs(SNAPSHOT_DIR, exist_ok=True)
                plt.imsave(os.path.join(SNAPSHOT_DIR, "found_frame.png"), img)
                print(f"[sar] 목표 확정 프레임 저장 — est={info['target_est']}")
            except Exception as e:
                print(f"[sar] found_frame 저장 실패: {e}")
        # 방문 확정별 프레임 — "무엇을 사과로 봤는가" 사후 검증의 핵심
        if info.get("visited", 0) != last_visited:
            last_visited = info["visited"]
            frame = getattr(mission, "_confirm_img", None)
            if frame is not None:
                try:
                    import matplotlib
                    matplotlib.use("Agg")
                    import matplotlib.pyplot as plt
                    plt.imsave(os.path.join(
                        SNAPSHOT_DIR, f"found_{last_visited}.png"), frame)
                    print(f"[sar] 방문 {last_visited} 확정 프레임 저장")
                except Exception as e:
                    print(f"[sar] 방문 프레임 저장 실패: {e}")
        if SNAPSHOT_EVERY_S > 0 and now - last_snap >= SNAPSHOT_EVERY_S:
            last_snap = now
            # 경량 렌더 — matplotlib figure는 제어 루프를 수백 ms 블록
            viz.save_fast(os.path.join(SNAPSHOT_DIR, "live_map.png"),
                          pose=pose, info=info)
        # 카메라 라이브 뷰 (블롭=노랑, YOLO=초록). YOLO는 후보가 보일 때만
        # 4초 간격으로 (CPU 추론 ~0.2s — 상시 돌리면 그 자체가 병목)
        if now - last_cam >= 1.0 and img is not None:
            last_cam = now
            if mission.detector.visible and now - last_yolo >= 4.0:
                last_yolo = now
                yolo_cache = mission.detector.yolo_boxes(img)
            viz.save_camera(os.path.join(SNAPSHOT_DIR, "live_cam.png"),
                            img, det=mission.detector.last,
                            yolo_boxes=yolo_cache)
        if info["state"] == Mission.DONE and not done_logged:
            done_logged = True
            print(f"[sar] MISSION DONE t={now:.1f}s — 정지 유지")
            io.drive(0.0, 0.0)


if __name__ == "__main__":
    main()
