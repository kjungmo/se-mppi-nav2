# 자율 모바일 로봇 플랫폼 — 전체 스택 아키텍처

> **작성일:** 2026-06-10 · **상태:** 캐논 설계 (살아있는 문서)
> **범위:** ROS2 Jazzy + Nav2 기반 자율 모바일 로봇(AMR) 소프트웨어 플랫폼의 전체 청사진.
> **원칙:** *차별화 지점은 직접 소유(IP·논문), 나머지는 성숙 스택에 얹는다.*
> **선행:** SOTA 서베이 · SE-MPPI 문제정의·설계 · Gazebo 라이브 런 핸드오프.

---

## 0. 왜 이 문서인가

원래 목표는 "**전체 로봇 시스템을 구축**"하는 것. 그러나 한 사람·한 프로젝트가 인지·SLAM·계획·제어·미션·학습을 *전부 바닥부터* 만드는 것은 불가능하고 불필요하다. 프로의 패턴은:

1. **차별화 층(differentiator)을 깊게 소유** → 논문/IP가 된다 (현재 **SE-MPPI** = 로컬 컨트롤러 + 동적 안전).
2. **나머지는 성숙한 오픈 스택을 차용·통합** (Nav2, ros2_control, SLAM Toolbox, robot_localization …).
3. 통합된 전체가 **실제로 한 몸으로 동작**하게 만들고(우리는 이미 Nav2 풀스택 위에서 컨트롤러를 교체·구동했다), 소유 층을 **하나씩 늘려간다**.

이 문서는 그 "전체 한 몸"의 지도와, 우리가 **무엇을 소유하고 무엇을 차용하는지**, 그리고 **다음에 무엇을 소유할지**를 못박는다.

---

## 1. 타깃 플랫폼 정의

- **대상 로봇:** 차동구동(differential-drive) AMR. 센서: 2D/3D LiDAR, RGB-D 카메라, IMU, 휠 엔코더.
- **임무 도메인:** 좁고 혼잡·동적인 실내(창고·병원·서비스). → SE-MPPI의 "탈출 + 동적 안전"이 정확히 여기서 값을 낸다.
- **런타임:** ROS2 Jazzy, 단일 로봇 → 추후 멀티로봇/플릿. 시뮬: Gazebo(gz sim 8) + 벤치마크(BARN/DynaBARN/HuNavSim).
- **비기능 요구:** 실시간 제어(≥10–20Hz), 형식적 안전(무충돌 보장), 재현가능·배포가능(Nav2-native 플러그인), 평가가능.

---

## 2. 층상 아키텍처 (12 레이어)

기호: ● 우리가 소유·구현 / ◑ 부분 소유 / ○ 차용(성숙 스택) / ░ 미착수

```
┌─────────────────────────────────────────────────────────────────────────┐
│ L12  인프라     lifecycle_manager · ParametersHandler · 로깅 · 배포(Docker) │ ○
│ L11  시뮬·평가  Gazebo · RViz · BARN/DynaBARN/HuNavSim · 메트릭 하니스        │ ◑
│ L10  학습       내비 파운데이션 모델 · RL · sim-to-real · 학습 예측           │ ░
│ L9   플릿·멀티  태스크 플래닝 · 멀티로봇 조정 · 스케줄링 · 교통관리            │ ░
│ L8   행동·미션  Behavior Tree(bt_navigator) · recovery · 미션 시퀀싱          │ ○
│ L7   안전       collision_monitor · ★CBF 안전필터★ · velocity_smoother · WD   │ ◑●
│ L6   로컬 제어  ★SE-MPPI 로컬 컨트롤러(MPPI+escape)★ · ros2_control            │ ●○
│ L5   전역 계획  global planner(NavFn/Smac) · route server · costmap           │ ○
│ L4   맵핑       occupancy/costmap · 시맨틱맵 · (3DGS/NeRF)                    │ ○░
│ L3   추정·측위  odometry · SLAM(Toolbox) · AMCL · EKF(robot_localization)     │ ○
│ L2   인지       ★동적장애물 검출·추적★ · 검출/세그 · 센서융합                 │ ◑░
│ L1   드라이버   LiDAR/카메라/IMU/엔코더 드라이버 · ros2_control HW             │ ○
└─────────────────────────────────────────────────────────────────────────┘
        ▲ 우리가 소유 중: L6(컨트롤러), L7(CBF), L2 일부(동적 트래커)
```

### 레이어별 명세 (목적 · 인터페이스 · 소유/차용 · 기여기회)

| L | 목적 | 핵심 ROS2 인터페이스 | 소유/차용 | 우리 기여 기회 |
|---|---|---|---|---|
| **L1 드라이버** | 센서·액추에이터 ↔ ROS2 | `/scan`, `/image`, `/imu`, `/joint_states`, `ros2_control` | ○ 차용(시뮬은 Gazebo) | 낮음 (표준) |
| **L2 인지** | 장애물·객체·동적성 추출 | in: `/scan`,`/points`,`/image` → out: `/detections`, `TrackedObstacles` | ◑ 동적 트래커는 소유, 검출·세그는 미착수 | **높음**: 학습기반 동적장애물 예측, 멀티모달 융합 |
| **L3 추정·측위** | map↔odom↔base TF, 자세 | in: 센서 → out: `/tf`, `/odom`, `/amcl_pose` | ○ SLAM Toolbox·AMCL·robot_localization | 중간: sandbox AMCL 안정화, SLAM 품질 |
| **L4 맵핑** | 정적/동적/시맨틱 맵, costmap | `/map`, `local/global_costmap` | ○ costmap_2d 차용 | 중간: 시맨틱·동적 costmap 레이어 |
| **L5 전역계획** | 시작→목표 경로 | `compute_path_to_pose` action | ○ NavFn/Smac 차용 | 낮음~중간 |
| **L6 로컬제어** | 경로 추종 cmd_vel | `follow_path` action → `/cmd_vel` | ● **SE-MPPI 소유** | **소유 완료**(차별화 핵심) |
| **L7 안전** | 무충돌 보장·속도 정형 | `/cmd_vel`→smoother→collision_monitor→`/cmd_vel_smoothed` | ●(CBF) ○(monitor/smoother) | **높음**: CBF↔monitor 통합, 형식적 보장 |
| **L8 행동·미션** | 임무 BT, recovery 오케스트레이션 | `navigate_to_pose`, BT XML | ○ bt_navigator 차용 | 중간: escape-aware recovery BT 노드 |
| **L9 플릿** | 다수 로봇 임무·교통 | 태스크/스케줄 API | ░ | **높음**(신규): 멀티로봇 escape 조정 |
| **L10 학습** | 예측·정책·파운데이션 | 모델 서빙 | ░ | **매우 높음**(신규): 동적예측·내비 FM |
| **L11 시뮬·평가** | 재현 벤치마크·메트릭 | Gazebo, 평가 하니스 | ◑ 프로토콜 설계됨, 하니스 미구현 | 중간: 자동 벤치마크 러너 |
| **L12 인프라** | 라이프사이클·파라미터·배포 | lifecycle, params, Docker | ○ | 낮음 |

---

## 3. 시스템 데이터 흐름 (런타임 한 사이클)

```
 센서(L1) ──┬──► 인지(L2): 클러스터·추적 ──► TrackedObstacles ─┐
            │                                                   │
            ├──► 측위(L3): AMCL/EKF ──► /tf (map→odom→base)      │
            │                                                   ▼
            └──► 맵핑(L4): costmap (정적+동적+inflation)         │
                         │                                      │
   목표 ──► 미션 BT(L8) ──► 전역계획(L5): global path ───────────┤
                                                                ▼
                        ┌──────────── 로컬 제어 (L6) ─────────────┐
                        │  SafeEscapeController : MPPIController   │
                        │   1) MPPI 샘플링 (+EscapeCritic: 탈출)   │
                        │   2) entrapment 감지 (단일 source)       │
                        │   3) 좌표(α) ← escape↔safety 조율        │
                        │   4) CBF 안전필터 (동적장애물, L7)        │
                        └───────────────┬─────────────────────────┘
                                        ▼  cmd_vel
            안전(L7): velocity_smoother → collision_monitor → /cmd_vel
                                        ▼
                                 드라이버(L1)/ros2_control → 바퀴
```

**우리가 소유한 경로**: 인지의 동적 트래커(L2) → 로컬 제어 전체(L6) → CBF 안전(L7)이 하나의 `nav2_se_controller` 플러그인 안에서 맞물린다. 나머지는 Nav2 표준.

---

## 4. 소유 vs 차용 — 결정과 근거

**소유(직접 구현·논문화):**
- **L6 로컬 컨트롤러 (SE-MPPI)** — 차별화 핵심. MPPI 위에 online 로컬미니마 탈출 + 동적 CBF + escape-safety 조율. *단일 선행연구 미점유 교집합* (novelty 검증 완료).
- **L7 CBF 안전필터** — look-ahead-point DCBF-QP. 컨트롤러와 한 몸.
- **L2 동적장애물 트래커(부분)** — costmap 클러스터+CV 추적. *향후 학습 예측으로 고도화 = 다음 논문 후보.*

**차용(성숙 스택에 얹기):**
- L1 드라이버/ros2_control, L3 SLAM Toolbox·AMCL·robot_localization, L4 costmap_2d, L5 NavFn/Smac, L8 bt_navigator, L12 lifecycle/params.
- 이유: 이들은 잘 검증돼 있고 차별화 포인트가 아니다. 재발명은 비용만 크다. **우리는 통합·플러그인으로 끼어든다.**

**원칙:** "Nav2-native 플러그인으로 배포가능"을 유지 → 차용 스택을 그대로 두고 우리 기여만 끼워 넣어, 연구가 곧 배포가능한 산출물이 된다.

---

## 5. 기여(논문/IP) 지도

| 후보 | 층 | 상태 | 비고 |
|---|---|---|---|
| **SE-MPPI** (escape + CBF + 조율, Nav2-native) | L6/L7 | 구현·단위검증·라이브로드 완료, 논문 초안 완료, 벤치마크 대기 | RA-L+ICRA 타깃 |
| 학습기반 **동적장애물 예측** → CBF 입력 (SE-Predict) | L2 | **N1+N2(고전)+N3 완료**(DOGM 분류 + 영속트랙·LS 예측기 + conformal q→시변 CBF 반경 + q-신뢰 escape 게이트, 226 tests) — 남은 것: N2 학습모델(GPU)·N4 라이브 A/B | `2026-06_se-predict-design.md` · tracker v2 + predictor + `conformal_calibrator` |
| **멀티로봇 escape 조정** (분산 CBF/우선권) | L9 | **N1+N2 코드 완료**(책임분배 CBF + `MultiRobotCoordinator` 이웃식별·우선권 플러그인 통합, 245 tests) — 라이브 멀티로봇 sim 검증만 대기 | proto + plugin · 조율의 *공간* 확장 |
| **내비 파운데이션 모델** ↔ 클래식 안전층 하이브리드 | L10 | **N1 프로토 검증 완료**(오라클 34% 개선·적대 제안 무충돌·silent degrade), N2 실모델 연결 대기 | `experiments/prototype/fm_shield_proto.py` · 조율의 *추상* 확장 |
| 시맨틱·동적 **costmap 레이어** | L4 | 미착수 | 사회적 비용/동적성 주입 |
| *(인프라)* **평가 하니스** (자동 벤치마크 러너) | L11 | **코어 구현·테스트 완료**(오프라인 51 tests), 라이브 런처만 GPU 대기 | `experiments/runner`+`experiments/analysis` · 설계 H-1~H-3·H-6 완료 |

> **N0 검증(2026-06-11):** 세 확장 트랙의 선행을 1차 검증(`2026-06_extension-tracks-n0-verification.md`) — 부품(conformal×CBF, 분담 CBF, FM 래퍼)은 기성, **차별점은 "조율"의 확장**으로 확정. 일반 주장("conformal CBF 최초" 등) 금지.
>
> **전략 — "조율"의 3축 확장 시리즈:** L6 SE-MPPI(완료)의 escape-safety 조율을 **시간**(L2 예측, 현재→미래) · **공간**(L9 멀티로봇, 자기↔타 로봇) · **추상**(L10 FM, 휴리스틱→의미) 으로 확장. 세 축 모두 **CBF 안전층이 불변의 토대**이고 그 위에서 제안만 풍부해진다 → SE-MPPI 강화 + 논문 시리즈. 인프라 L11(하니스)이 모든 정량 평가를 공유.

---

## 6. 로드맵 (단계별)

**Phase A — 차별화 층 완결 (현재):**
- SE-MPPI: M6 벤치마크 평가(BARN/DynaBARN/HuNavSim) + 탈출 라이브 시연 + 논문. → "완결된 기여" 1건 확보.

**Phase B — 전체 한 몸 통합 검증:**
- Nav2 풀스택(L1–L8) + SE-MPPI를 *하나의 재현가능 배포*로 패키징. 실로봇/시뮬에서 임무 단위 E2E 데모. (인프라 L12 정리: launch·params·Docker.)

**Phase C — 다음 소유 층 (택1, §5 기여지도 기준):**
- 1순위 **L2 학습 동적장애물 예측** → SE-MPPI의 CBF/TTC 입력 고도화 (소유 층 강화 + 새 논문).
- 대안 **L10 FM↔안전 하이브리드** 또는 **L9 멀티로봇**.

**Phase D — 플랫폼화:**
- 평가 하니스(L11) 자동화, 시맨틱/동적 맵(L4), 미션·플릿(L8/L9)로 "플랫폼" 폭 확장.

각 Phase는 **(a) 차용 스택 위에서 (b) 소유 층 하나를 추가/강화하고 (c) 전체가 한 몸으로 도는지 E2E로 검증**하는 동일 리듬을 따른다.

---

## 7. 현재 좌표 (2026-06)

- **소유·동작 확인:** L6 SE-MPPI + L7 CBF + L2 동적 트래커 — 실 ROS2 Jazzy+Nav2+Gazebo 스택에 로드·활성화·cmd_vel 생성(라이브 핸드오프 문서 참조). EscapeCritic 네임스페이스 통합 갭 수정 완료. **논문 초안 완료**(`docs/papers/2026_se-mppi-paper-draft.md`, 벤치마크 수치만 *pending*).
- **차용·동작 확인:** L1·L3·L4·L5·L8·L12 — Nav2 풀스택으로 라이브 구동 중.
- **설계 완료·구현 대기:** L11 평가 하니스(`2026-06_evaluation-harness-design.md`) · L2 SE-Predict(`2026-06_se-predict-design.md`).
- **다음 행동:** ① L11 하니스 구현 → SE-MPPI 논문 *pending* 표 측정·확정 (Phase A 종결). ② 병행: L2 N1(고전 DOGM 정적/동적 분류 — 라이브 런 벽-freeze의 근본 해결).

> 이 문서는 살아있는 캐논. 새 층을 소유하거나 결정이 바뀌면 §2 표·§5 지도·§6 로드맵을 갱신한다.
