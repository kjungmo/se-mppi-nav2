# SE-MPPI Paper 1 — Figure Manifest

> 논문(`../2026_se-mppi-paper-draft.md`)의 그림 ↔ 소스 ↔ 상태 매핑.
> **Paper 1 기여는 C1–C4뿐**(conformal·학습예측은 Paper 2 / multi-robot은 Paper 3).
> 따라서 `fm_shield.png`·`multirobot_*.png`는 Paper 1 그림이 **아님**(아래 분리 표기).

## Paper 1 그림

| 그림 | 본문 위치 | 소스 파일 | 생성 스크립트 | 상태 |
|---|---|---|---|---|
| **Fig. 1 — Architecture / system data-flow** | §IV-A (개요), §I 언급 | `architecture.mmd` → `architecture.png` | Mermaid (텍스트 소스) | **new** — `.mmd` 신규 작성 + `architecture.png` 렌더 완료(`npx @mermaid-js/mermaid-cli`). 본문은 `.png` 참조 |
| **Fig. 2 — U-trap escape** | §VI-A | `../../../experiments/prototype/figures/utrap_escape.png` | `experiments/prototype/run_validation.py` → `se_mppi_proto.py` | **exists** (paper-quality 라벨 필요 시 regen) |
| **Fig. 3 — Dynamic-obstacle CBF avoidance** | §VI-A | `../../../experiments/prototype/figures/dynamic_cbf.png` | `experiments/prototype/run_validation.py` → `se_mppi_proto.py` | **exists** |
| **Fig. 4 — Escape-safety coordination (α 상승 + slack≈0)** | §VI-A (핵심, §IV-E 명제 입증) | `../../../experiments/prototype/figures/coordination.png` | `experiments/prototype/run_validation.py` → `se_mppi_proto.py` | **exists** |

## 렌더 / 재생성 방법

- **architecture.mmd → PNG** (텍스트 소스, GPU 불필요):
  ```bash
  npx -y @mermaid-js/mermaid-cli -i docs/papers/figures/architecture.mmd \
      -o docs/papers/figures/architecture.png --scale 7
  ```
  (이 컨테이너엔 `mmdc` 미설치 — 워크스테이션 또는 npx로 1회 렌더.)
  `--scale 7` 은 인쇄 해상도 확보용(0.85\textwidth 배치에서 ~900 DPI).
  렌더 후 `docs/papers/latex/figures/architecture.png` 에 동일 파일을 복사해
  두 사본을 바이트 단위로 일치시킨다(현재 5488×1078, subgraph `direction LR`).
- **prototype PNG 재생성** (Python+matplotlib+osqp, GPU 불필요이나 conda env 필요):
  ```bash
  micromamba run -n ros2 python3 experiments/prototype/run_validation.py
  ```
  현 PNG는 이미 존재하고 캡션은 그림을 *서술*만 하므로 **재생성은 선택**.
  paper-quality 라벨(폰트·축 단위)이 필요할 때만 워크스테이션에서 regen → **needs-regen-on-workstation**.
  > 주의: 재생성 시 출력이 `experiments/`로 쓰이므로(다른 에이전트 소유) 이 repo에선 회피.

## Paper 1 범위 밖 그림 (다른 논문 — 혼동 방지)

| 그림 | 귀속 | 소스 | 생성 스크립트 |
|---|---|---|---|
| `fm_shield.png` | **Paper 2 (SE-Predict / FM-Shield)** | `experiments/prototype/figures/fm_shield.png` | `run_fm_shield_validation.py` → `fm_shield_proto.py` |
| `multirobot_corridor.png` | **Paper 3 (Multi-SE-MPPI)** | `experiments/prototype/figures/multirobot_corridor.png` | `run_multirobot_validation.py` → `multi_se_proto.py` |
| `multirobot_intersection.png` | **Paper 3 (Multi-SE-MPPI)** | `experiments/prototype/figures/multirobot_intersection.png` | `run_multirobot_validation.py` → `multi_se_proto.py` |

## 캡션 출처 / 정직성

캡션은 그림이 *보여주는 것*만 서술한다. §VI-C의 대규모 벤치마크 수치는 **미측정(pending)**
이며 어떤 그림 캡션도 벤치마크 수치를 인용하지 않는다. §VI-A 메커니즘 결과(U-trap 완주,
CBF 회피, α 2→6 / slack≈0)는 prototype에서 **실측**된 것이다.
