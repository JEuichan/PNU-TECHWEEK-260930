<br>

<h1 align="center">
  2026 부산대학교 TECH WEEK:<br>
  Autonomous Mobile Robot의 Search & Rescue Mission
</h1>
<br>

---

# SAR Pipeline (`sar-pipeline` 브랜치)

원본 저장소의 Webots 환경(TurtleBot3 Burger + apartment/breakroom 월드) 위에서
**Search & Rescue 미션을 자율 수행하는 파이프라인** 구현 브랜치입니다.

> **미션**: 사전 지도 없이 미지의 환경을 탐색 → **빨간 사과 2개를 각각
> 탐지·접근** → 시작 지점으로 복귀. 정적 장애물·이동 보행자와 충돌 금지.

## 실행 방법

### 준비 (1회)

```bash
# Python 3.10+ (개발은 3.12, 코드는 3.10 호환 문법만 사용)
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt        # numpy, matplotlib(선택)
.venv/bin/pip install ultralytics                # 선택: YOLO 디코이 베토
```

`controllers/sar_controller/runtime.ini` 의 파이썬 경로를 **자기 머신의
venv 경로**로 수정 (Webots가 컨트롤러를 그 파이썬으로 실행):

```ini
[python]
COMMAND = /절대경로/.venv/bin/python
```

### Webots 실행 (아파트 2사과 미션)

```bash
webots worlds/apartment_sar.wbt        # 실시간 GUI
# 헤드리스: webots --batch --mode=fast --minimize --no-rendering worlds/apartment_sar.wbt
```

- 미션 파라미터는 `config_override.json` (시작 포즈, 목표 색/개수, YOLO on/off)
- 실시간 지도: `live_view.html` 을 브라우저로 열면 `out/live_map.png` 가
  2초마다 갱신됨. 목표 확정 순간의 카메라 프레임은 `out/found_frame.png`
- 연습용 소형 월드: `worlds/practice_arena.wbt`

### Webots 없이 검증 (오프라인 시뮬레이터)

```bash
.venv/bin/python run_tests.py                    # 단위 테스트 (13개)
.venv/bin/python sim_offline.py --scenario rooms_two   # 2사과 E2E 미션
.venv/bin/python sim_offline.py --sweep          # 6개 지형 × 사람 × 시드 매트릭스
.venv/bin/python sim_offline.py --snapshots      # out/에 지도 PNG 저장
```

## 아키텍처

```
센서 → PoseEstimator(인코더+컴퍼스 캘리브레이션) → OccupancyGrid(동적 빔 제외)
     → 색 탐지(HSV 블롭+기하 게이트) → 위치 추정(LiDAR 일관성/단안 폴백)
     → Mission 상태기계(SPIN→EXPLORE→SEEK→GOTO→…→RETURN)
     → A*(사람 캡슐·영토 페널티) + pure pursuit → 정적 안전 클램프 → (v, ω)
```

| 모듈 | 역할 |
|---|---|
| `sar/config.py` | 모든 튜너블 — **당일엔 사실상 이 파일만 수정** |
| `sar/odometry.py` | 차동구동 odometry + Webots 컴퍼스 부호 보정·오프셋 캘리브레이션 |
| `sar/mapping.py` | log-odds Occupancy Grid (벡터화, 동적 빔 제외 통합) |
| `sar/detection.py` | HSV 블롭 + 디코이 방어 게이트 + 단안 거리 + YOLO 베토 |
| `sar/exploration.py` | frontier 클러스터·점수화(사람 방향 후순위)·관성·블랙리스트 |
| `sar/planning.py` | A*(코너컷 금지·예측 캡슐 페널티) + 추종 + 충돌 코리도 안전 클램프 |
| `sar/state_machine.py` | 다중 목표 미션 로직 + 동적 장애물 분류 + recovery |
| `controllers/sar_controller/robot_io.py` | Webots API 격리층 (디바이스 자동탐색·자동캘리브레이션) |
| `sim_offline.py` | Webots 없는 E2E 가상 시뮬레이터 + 시나리오 스윕 하네스 |

## 핵심 설계 결정 (실측 근거)

1. **사과는 LiDAR에 안 보인다** — 반경 5cm 사과는 LDS-01 스캔 평면(~17cm)
   아래. 거리 추정은 **blob 크기 기반 단안**이 주력, LiDAR는 "그 거리면
   blob이 이 크기여야 함" 일관성 검사 통과 시에만 사용.
2. **Webots 컴퍼스는 부호가 반대다** — 컴퍼스는 북쪽 벡터를 로봇 프레임에서
   반환하므로 `atan2(v[1],v[0]) = 북쪽각 − θ` (기울기 −1). 부호 반전 후
   시작 헤딩(제공값)으로 오프셋 캘리브레이션. *(멀티에이전트 리뷰가 발견,
   codex 교차검증 — 시뮬레이션으로는 탐지 불가능한 결함이었음)*
3. **디코이 방어 다층 게이트** — 아파트 월드의 빨간 디코이들을 실측으로
   각개 격파: 소화기(화면 상단 접촉=키 큰 물체 기각), 라디에이터 노브
   (채도 s≥0.60 — 순색 사과 S≥0.65 vs 재질색 ≤0.54 실측 분리),
   OrderSign(수평선 위 블롭 기각), 소다캔(YOLO 베토 + 확정단계 종횡비).
   **yolo11n은 Webots 사과를 'apple'로 인식 못 하므로**(실측) YOLO는
   양성 확인이 아니라 **거부권(베토)**으로만 사용.
4. **가려진 사과 대응 2단계 게이트** — 탐지 단계는 느슨(반달형도 SEEK 추적),
   **위치 확정은 온전한 원형일 때만**(가림 상태의 단안 거리는 2배 오차).
5. **이동 보행자 스택** — §"움직이는 장애물" 참고. 사람이 로봇보다 빠를
   수 있다는 전제의 조기 양보/커밋/탈출 + 경로 차원의 예측 회피.
6. **문 통과** — 각도 부채꼴이 아닌 **충돌 코리도**(진행 폭 안의 물체만)
   판정 + 정지 히스테리시스 + 조향-속도 연동 + 명령 슬루 제한
   (Nav2 RPP/velocity smoother 문헌 기반).
7. **갇힘 탈출 스택** — 탁자 다리/가구 사이에 갇히는 케이스 전용:
   - **소프트 팽창 완화**: 표준 팽창(로봇반경+7cm)으로 경로가 전멸하면
     팽창 여유 halo를 고비용 통과 허용으로 A* 재시도 — 물리적으로
     지나갈 수 있는 좁은 틈이면 경로가 나온다 (좁은 틈 추종 중엔 회피
     마진 일시 완화 + 저속 상한 0.09m/s)
   - **틈 중앙 직진 탈출**: 연속 free 빔 구간(run)의 한가운데를 겨냥해
     수직 통과 (섹터 최소거리 방식은 문설주가 점수를 깎아 "먼 벽
     방향"=장애물 쪽을 고르는 오류) + 코리도식 중앙 유지 조향
   - **유령 동적 장애물 소멸**: 문설주 스침 빔이 '정지한 사람'으로
     오분류되어 탈출·복구를 영구 차단하는 3중 데드락 차단 (3초간
     0.12m 미만 이동 = 유령 → 추적 해제)
   - **왕복 진동 감지**: 12초간 0.7m 안에서 맴돌면 갇힘 판정 (기존
     5초/8cm 게이트는 30cm 왕복·소형 원돌기를 놓침)
8. **도주 방향 점수화** — 보행자 도주는 수직 이탈 > 선로 횡단 > 전진
   도주 > 후진 (후진 도주는 0.85×max_v < 보행자 0.2m/s로 필패 수라
   강한 감점, 여유 점수 상한 1.2로 "넓다는 이유만으로" 후진이 이기는
   것 방지) — 이 수정으로 corridor 시나리오 최초 무충돌 완주.

## 움직이는 장애물(보행자) 처리 방식

1. **분류**: LiDAR 빔 끝점이 "이미 확실히 빈공간으로 알던 셀"(log-odds<−1.2,
   벽 이웃 아님)에 찍히면 동적 → 위치 연속성(직전 사람 위치 0.45m 내)으로 보강
2. **지도 보호**: 동적 빔은 지도에 통합하지 않음 (사람 잔상이 지도를
   오염시키면 분류 자체가 무너지는 악순환 차단)
3. **추정**: 좁은 각도 클러스터(≥3빔)의 중심 = 위치, 0.7s 창 차분 = 속도
4. **경로 차원 회피**: A*에 사람 캡슐(현위치+속도×2.5s 쓸기) + 영토(다녀간
   자리) 페널티 → 경로가 애초에 사람 동선을 피해 나감. frontier 점수도
   사람 방향 후순위 (반대쪽부터 탐색)
5. **반응 차원 회피**: 접근 중이면 조기 양보(0.95m) → 멀어지면 커밋 창으로
   전속 횡단 → 근접(0.5m) 시 사람 속도벡터 기준 "선로 수직 이탈"(여유
   공간 점수화) → 초근접 실명 래치. 비껴가는 궤적(miss-distance>0.5m)은
   양보 없이 통과. **모든 동적 제안 위에 정적 안전 클램프가 최종 적용**

## 검증 상태

- **오프라인 스윕**: 7개 지형(방·2사과·복도·개방·미로·탁자우리) ×
  사람(실측 0.2m/s) 유무 — 핵심 시나리오 **전부 무충돌 완주**:
  rooms_two(2사과) 186s, corridor+사람 255s(최초 완주), table(0.34m
  틈 하나뿐인 우리 탈출→목표→재진입 복귀) 647s
- **Webots 실전**: apartment 월드 E2E 완주 (디코이 4종 실측 차단),
  practice_arena 104.9s 완주
- **품질 공정**: 오프라인 스윕으로 회귀 검증 → 멀티에이전트 적대 리뷰
  (23 에이전트, 치명 결함 3건 발견) → codex 교차검증 → 실기 스모크

## 당일 체크리스트

`docs/DAY_OF_CHECKLIST.md` — 계획안 준수 매트릭스, LiDAR 방향 30초 검증,
HSV 보정 절차, 흔한 증상→원인 표, 제출 전 동결 절차.

## 알려진 한계

- 로봇(0.21m/s)보다 2배 빠른 왕복 차단자는 물리적으로 회피 보장 불가
  (스트레스 케이스로만 유지 — 실전 보행자는 0.2m/s)
- 지도 우선 모드(`mission.map_first`)·벽 추종(`explore.wall_follow`)은
  옵션으로 남겨둠 — 기본은 목표 우선(frontier) 모드
- green 사과는 텍스처 기반이라 HSV 범위가 넓음 — 당일 실화면 보정 필요
- 목표가 책상 위에 있는 규칙이면 "수평선 위 기각" 게이트 완화 필요
