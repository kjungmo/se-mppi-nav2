# SE-MPPI 백로그 (Deferred work)

진행 중 미뤄둔 항목을 추적. 완료 시 제거하고 해당 마일스톤 문서에 반영.

## M1.x — EscapeCritic 고도화 (탈출 로직 강화)
> 현재 M1은 **코스트맵 cost-proxy repulsion** + progress-stall 감지로 동작(테스트 통과).
> 아래는 비볼록/동적 환경에서 탈출 신뢰도를 높이기 위한 후속.

- [x] **진짜 거리장 APF (완료 2026-06-08)**: `obstacleDistanceField`(multi-source
      Dijkstra, octile) + `computeApfRepulsionCosts`로 고전 APF
      `U_rep = 0.5·η·(1/d − 1/d0)²` (d<d0) 구현. EscapeCritic에 `use_apf` 토글
      (APF↔cost-proxy ablation). 단위테스트 4개(거리장 2 + APF 2) 통과.
- [x] **자유공간 gap 탐색 (완료 2026-06-08)**: `findEscapeGap`(raycast로 goal에
      가장 가까운 viable 개구부 탐색) + `computeGapAttractionCosts`(개구부 방향
      정렬 궤적 보상)로 임시 attractive subgoal 구현. EscapeCritic에 통합
      (`use_gap_search` 토글). 단위테스트 4개 통과. → **M1.x 완료**(APF + gap).
- [ ] **감지 신호 보강**: 현재 progress-stall 단일 신호. 코스트맵 기반
      "목표 방향 차단" 신호 추가(목표 방향 best-rollout이 장애물벽에 막힘).
      주의: Jazzy `CriticData`에 `trajectories_in_collision` 없음(main 브랜치만) →
      `costs` 누적값은 critic 순서 의존이라 신뢰 낮음, 코스트맵 직접 질의 권장.
- [ ] **단위테스트**: U자 트랩 합성 코스트맵에서 탈출 궤적이 저비용이 되는지.

## 기타
- [ ] `DynamicObstacleTracker` 실구현(M2.x): 코스트맵 lethal 클러스터링 +
      프레임간 CV 속도추정. (M2는 CBF-QP 필터 코어부터)
- [ ] BARN ROS1→ROS2 브리지 PoC (평가 tier 1).
- [ ] async hook 옵션 검토(세션 시작 지연 vs race) — 현재 sync.

## M5b.x — loopback 풀-드라이브 마무리
> M5b에서 컨트롤러가 live Nav2에 로드·설정·활성화되고 목표를 수락해 제어루프가
> 도는 것까지 확인됨. 후속 진단(2026-06-08)으로 추가 확인:
>
> - **맵 좌표 해결**: tb3_sandbox 맵 분석(침식 후 최대 연결 free 영역)으로
>   start(-0.3,0.25)·goal(0.9,-2.25) 확보 → **planner 경로 생성 성공**(dist≠0).
>   맵 중심 (0,0)은 unknown 셀이라 기존 실패 원인이었음.
> - **컨트롤러는 정상**: `/cmd_vel_nav`(컨트롤러 출력) = 0.096 m/s 전진,
>   로봇 실제 이동 확인. **stock MPPI(se_enabled=false)도 동일 거동** → 미수렴은
>   컨트롤러 무관.
> - **남은 병목**: `/cmd_vel_nav` 0.096 → `/cmd_vel` 0.002 로 throttle.
>   collision_monitor FootprintApproach 비활성화해도 잔존 → **velocity_smoother +
>   좁은 sandbox에서 MPPI 회전과다(기본 params 튜닝)** 문제.

- [ ] 완주 미해결 — **RoboStack loopback 플러밍 의심**: warehouse(넓은 맵)에서도
      dist가 감소가 아닌 *증가*(로봇이 목표 반대로 표류). `/odom` 미발행(hz 빈값),
      `/cmd_vel_nav`(0.096) → `/cmd_vel`(0.002) throttle. velocity_smoother는
      OPEN_LOOP라 직접 원인 아님 → loopback의 odom/속도 피드백 플러밍 또는
      Nav2 cmd_vel 체인 문제로 추정. **stock MPPI도 동일** → 컨트롤러 무관.
- [x] **baseline 확정(2026-06-08)**: 수정 없는 stock nav2_params로 tb3_loopback
      데모를 그대로 실행해도 로봇 미주행(dist 3.63m 고정), `/odom` 미발행.
      stock `/cmd_vel_nav`=0.13 m/s 출력되나 로봇 위치 불변. → **완주 실패는
      RoboStack loopback 시뮬 env 문제로 확정, 우리 컨트롤러와 완전 무관.**
- [ ] 후속(완주가 꼭 필요할 때): **(b) Gazebo 물리 시뮬(headless+xvfb)** 시도함(아래) —
      현재 컨테이너로는 미완. 정상 ROS2 워크스테이션 또는 GPU 머신 필요.

### ④ Gazebo 물리 시뮬 시도 결과 (2026-06-08)
> loopback이 막혀 Gazebo 물리로 전환. **물리 시뮬 자체는 헤드리스로 구동됨**:
> - gz sim 8 + waffle + tb3_sandbox 물리 월드가 software GL(llvmpipe)로 기동,
>   **RTF≈1.2(거의 실시간)**, gz 크래시 없음. 라이다 `/scan`도 실제 발행됨.
> - SafeEscapeController 로드, AMCL이 초기 pose 수락(initialPoseReceived).
>
> **그러나 완주 실패 — 원인 확정(컨테이너 GPU 부재):**
> - software-rendered 라이다가 **너무 느려** AMCL이 제때 TF 변환/로컬라이즈 못 함
>   (`amcl: Failed to transform initial pose in time` 반복).
> - → map→odom TF 부재 → **global_costmap 활성화 60초 타임아웃**
>   (`global_costmap: Failed to activate`) → `lifecycle_manager_navigation:
>   Failed to bring up` → nav 영영 비활성 → 주행 불가.
> - **우리 컨트롤러 무관** — Nav2 활성화 자체가 센서 TF 타이밍에서 실패.

- [ ] 정상 ROS2 워크스테이션/GPU에서 Gazebo 완주 + escape·CBF 정량 로깅(궤적/α/slack/충돌거리).
- [ ] (컨테이너 시도 시) 완화책: `lifecycle bond_timeout`·costmap `transform_tolerance`
      대폭 상향으로 느린 라이다에 여유 → 활성화는 될 수 있으나 제어 루프도 느려짐(미검증).
- [ ] 완주 후 escape·CBF 동작 정량 로깅(궤적/α/충돌거리).
- 스모크 자산: `experiments/sim/{nav2_se_loopback.yaml, smoke_drive.py}`
      (collision_monitor FootprintApproach 비활성; 좌표는 SE_START/GOAL env로 지정).
- 맵 free-좌표: tb3_sandbox start(-0.3,0.25)/goal(0.9,-2.25),
      warehouse start(0.23,0.26)/goal(0.23,5.24).


