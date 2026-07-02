# CLAUDE.md — se-mppi-nav2

**SE-MPPI**(Safe-Escape MPPI) — ROS2 Nav2-native 로컬 컨트롤러 — 의 연구·구현·논문
전용 repo. (SLAM·맵핑 등 다른 도메인 연구는 별도 repo에서 진행.)

## 빌드 환경 (중요)

이 저장소는 **ROS2 Jazzy + Nav2** 패키지(`src/nav2_se_controller`)를 빌드한다.
Claude Code on the web 컨테이너에는 ROS2가 없고 `packages.ros.org`가 네트워크
정책으로 차단(403)되어 있다. 따라서 **RoboStack(conda-forge)** 로 ROS2를 설치한다
(conda-forge/robostack-jazzy/github/pypi는 접근 가능).

### 환경 준비 (세션마다 1회)

```bash
bash scripts/setup_ros2_env.sh        # micromamba + ROS2 Jazzy + Nav2 + Gazebo 설치(멱등)
```

컨테이너 상태는 hook/설치 완료 후 캐시되므로, 최초 1회만 오래 걸리고 이후엔
ready marker로 빠르게 통과한다. 설치 위치: `/opt/micromamba/envs/ros2`.

### 빌드 / 실행

활성화 후(또는 `micromamba run -n ros2`로) colcon 사용:

```bash
export MAMBA_ROOT_PREFIX=/opt/micromamba
micromamba run -n ros2 bash -lc '
  cd /home/user/robotics_engineering
  colcon build --packages-select nav2_se_controller
  source install/setup.bash
'
```

설치 검증됨(2026-06): `ros2`(Jazzy), `colcon`, `gcc 14.3`, `cmake 4.3`,
`gz sim 8.10`, `nav2_mppi_controller` 헤더, `osqp-eigen` 모두 사용 가능.

> `build/`, `install/`, `log/`, conda env는 `.gitignore` 처리. 커밋하지 말 것.

## 프로젝트 구조

```
docs/research/       # SOTA 서베이, 문제정의·선행연구
docs/architecture/   # 아키텍처 설계
src/nav2_se_controller/   # ROS2 Nav2 플러그인 (EscapeCritic + 추후 CBF 필터)
scripts/             # 환경 설치 스크립트
experiments/         # 평가 (BARN/DynaBARN/HuNavSim)
```

핵심 문서:
- `docs/research/2026-06_mobile-navigation-sota-survey.md` — 내비게이션 SOTA
- `docs/research/2026-06_safe-escape-mppi-problem-statement.md` — 문제정의·기여
- `docs/architecture/2026-06_safe-escape-mppi-design.md` — 컨트롤러 설계

## SE-MPPI 핵심

MPPI(Nav2 기본급 SOTA 로컬 컨트롤러)의 두 약점 — 로컬미니마 함정, 형식적 안전
부재 — 을 함께 해결. 기여: **온라인 로컬미니마 감지·탈출 + 동적장애물 CBF 안전필터
+ escape-safety 조율**을 단일 Nav2-native 플러그인으로. (단일 선행연구 미점유 교집합)

## 작업 규칙

- 브랜치 정책: 원격은 `main` 단일 브랜치 (2026-07-03 히스토리를 단일 커밋으로
  재구성). 작업 브랜치는 머지 후 삭제.
- 구현 인터페이스는 **설치된 Jazzy 버전 헤더** 기준 (예: critic `costs`는 xtensor,
  `main` 브랜치의 Eigen과 다름). 추측 말고 헤더 확인.
- 리서치 수치·논문 인용은 신뢰도 표기 유지(자체보고/미확인).
