# 모바일 로봇 내비게이션 SOTA 서베이 (2024–2026)

> **작성일:** 2026-06-08
> **목적:** 이동·내비게이션 스택의 최신 기술·논문·오픈소스·벤치마크를 정리하여, 이어지는 **소프트웨어 아키텍처 설계 및 구현**의 기반 문서로 삼는다.
> **방법:** 5개 검색 축(학습기반 내비 / SLAM·맵핑 / 플래닝 / 플랫폼·Nav2 / 벤치마크·트렌드)에 대해 병렬 웹 리서치를 수행하고, 결과를 교차검증·중복 병합하여 종합.
> **신뢰도 표기:** 잘 확립된 사실은 그대로, 단일 논문 자체보고 수치는 *[자체보고]*, 검증 못 한 항목은 **[미확인]**으로 표시. 2026년 초 arXiv ID(2601.x~2603.x)는 매우 최신 프리프린트라 일부는 원문 확인이 어려웠음.

---

## 0. Executive Summary — 엔지니어를 위한 한눈 요약

내비게이션 스택을 **4개 레이어**로 보면 현재 SOTA와 "실제 쓸 수 있는 것"은 다음과 같다.

| 레이어 | 실전 검증된 선택지 (지금 바로 구축) | 최신 연구 프론티어 (실험적) |
|---|---|---|
| **상태추정·맵핑** | LiDAR-(I)O: FAST-LIO2 / KISS-ICP, slam_toolbox(2D), robot_localization EKF | 3D foundation SLAM (MASt3R-SLAM, VGGT-SLAM), 3DGS SLAM, PIN-SLAM |
| **전역 계획** | Nav2 Smac (2D A* / Hybrid-A* / State Lattice), Route Server | 학습기반·세만틱 토포메트릭 플래닝 |
| **지역 제어** | Nav2 **MPPI** (현 기본 고성능), Regulated Pure Pursuit, DWB/TEB | CBF 안전필터 + MPPI, DRPA-MPPI(로컬미니마 탈출) |
| **인지·의미** | 코스트맵 레이어, Collision Monitor | 오픈보캐뷸러리 맵(ConceptGraphs→OpenVox), VLA/VLN |
| **오케스트레이션** | Nav2 Behavior Trees, Open-RMF + VDA5050 (플릿) | 내비게이션 foundation/world model |

**핵심 흐름 3가지:**
1. **3D foundation model이 SLAM 프런트엔드를 대체** — MASt3R/VGGT 같은 feed-forward 3D 재구성 prior가 dense SLAM을 구동. 단, metric scale·투영 모호성·장기 시퀀스 확장이 미해결.
2. **VLM/VLA가 내비게이션으로 수렴** — NaVILA·Uni-NaVid·StreamVLN처럼 "중간 레벨 언어 명령 → RL locomotion" 2단 구조가 다족/휠 로봇의 주류 레시피로. 다만 실시간 엣지 추론(10–20Hz vs 제어 50–100Hz)이 병목.
3. **하이브리드(고전+학습)가 실배포·대회를 지배** — 순수 end-to-end가 아니라 Nav2/Smac+MPPI 같은 고전 스택에 학습·안전필터를 얹는 방식이 BARN 우승·산업 배포의 현실.

> **신뢰도 캐비엇:** 모든 벤치마크 수치는 논문/프로젝트 페이지 자체보고이며 독립 재현된 값이 아니다. 교차 데이터셋 비교는 사과-오렌지가 많다(예: VSLAM-LAB는 ORB-SLAM2가 KITTI에서 여전히 DROID-SLAM·MASt3R-SLAM을 능가함을 보고 — 방법 우열은 조건 의존적).

---

## 1. SLAM · 인지 · 맵핑

### 1.1 Visual SLAM

**고전 베이스라인 — ORB-SLAM 계열**
- **ORB-SLAM3** (2021, Campos et al., Univ. of Zaragoza; IEEE T-RO). Feature 기반 멀티맵 visual-inertial SLAM. 텍스처 좋은 쉬운 시퀀스에서는 여전히 최강 베이스라인이며 DROID-SLAM·MASt3R-SLAM보다 우수, 저텍스처/격렬 모션에서 열화. (VSLAM-LAB 벤치마크 2025: https://arxiv.org/html/2504.04457v1)
- 프런트엔드 업그레이드: **SELM-SLAM3**(SuperPoint+LightGlue), 어려운 시퀀스에서 ORB-SLAM3 대비 평균 ~87.84% 개선 *[자체보고, 단일논문]*.

**딥 기반 — DROID-SLAM 계열**
- **DROID-SLAM** (2021, Teed & Deng, Princeton; NeurIPS). Optical flow 기반 recurrent dense BA. 어려운 모션/저텍스처 강함, 쉬운 장면은 ORB-SLAM2보다 약함. (https://arxiv.org/pdf/2108.10869)
- **DPVO** (Deep Patch VO) — sparse-patch 경량 후속.

**3D foundation model SLAM — 2025–2026 지배적 트렌드**
- **MASt3R-SLAM** (CVPR 2025, Murai, Dexheimer, Davison — Imperial College London). MASt3R two-view 3D 재구성 prior 위에 구축한 실시간 dense monocular SLAM, 고정 카메라 모델 불필요. ~15 FPS(RTX 4090), 전역 일관 pose + dense geometry. 벤치: TUM-RGBD, 7-Scenes, EuRoC, ETH3D-SLAM. **코드(MIT):** https://github.com/rmurai0610/MASt3R-SLAM · 논문: https://arxiv.org/abs/2412.12392 · ROS 공식 없음.
- **VGGT-SLAM** (NeurIPS 2025, MIT AeroAstro). VGGT feed-forward 재구성기로 submap을 점진적·전역 정렬, 비보정 카메라를 15-DoF 투영(SL(4) manifold) 모호성 하에 정렬. https://arxiv.org/abs/2505.12549
- 후속 efficiency/consistency 패밀리: **VGGT-SLAM++**, **EC3R-SLAM**, **HyVGGT-VO**, **Spann3R**(스트리밍 메모리 모듈). 큐레이션: https://github.com/3D-Vision-World/All-3R-SLAM-in-this-Repo
- 동적 환경: **VAR-SLAM** (2025).

### 1.2 LiDAR · LiDAR-Inertial(-Visual) SLAM

**확립된 SOTA 베이스라인**
- **FAST-LIO2** (2022, Xu/Cai/Zhang et al., HKU MaRS Lab; IEEE T-RO). 직접 point registration + ikd-Tree + iterated EKF. 대부분 후속의 레퍼런스 LIO 아키텍처.
- **Point-LIO** (HKU MaRS) — point-by-point, 포화 IMU/격렬 모션 대응.
- **KISS-ICP** (2023, Vizzo/Guadagnino et al., PRBonn; IEEE RA-L). 파라미터 거의 없는 point-to-point ICP LiDAR 오도메트리, 일반화 강함. https://arxiv.org/pdf/2503.12660

**최신 (2024–2026)**
- **FAST-LIVO2** (IEEE T-RO 2025, HKU MaRS). 단일 ESIKF로 IMU+LiDAR+이미지 융합하는 직접식 LiDAR-Inertial-Visual 오도메트리, degraded 환경 강건, mesh/NeRF 렌더 지원. **코드(GPLv2):** https://github.com/hku-mars/FAST-LIVO2 · **ROS2 커뮤니티 포트:** https://github.com/VIS4ROB-lab/FAST-LIVO2-ROS2
- **KISS-SLAM** (2025, PRBonn). KISS-ICP + 로컬 맵핑 + 루프클로저 + pose-graph. 최소 튜닝, 교차 데이터셋 일반화 강함. (ROS 성숙도 미확인) https://arxiv.org/pdf/2503.12660
- **PIN-SLAM** (IEEE T-RO 2024, Yue Pan et al., PRBonn). 최초의 본격 **implicit-neural** LiDAR SLAM(elastic neural points), 오도메트리+neural 루프클로저+pose-graph, 정확한 메시 생성. **코드:** https://github.com/PRBonn/PIN_SLAM · https://arxiv.org/abs/2401.09101
- 기타: **COIN-LIO**(intensity 증강, degenerate 장면), **D²-LIO**(방향성 degeneracy), **OKVIS2-X**(2025, dense depth/LiDAR/GNSS 설정 가능), **GenZ-LIO**(실내외 경계 일반화) *[2026 프리프린트]*.

### 1.3 3D Gaussian Splatting (3DGS) · NeRF SLAM/맵핑

**개척자 (2023–2024)**
- **SplaTAM** (CVPR 2024, Keetha et al., CMU+MIT). 3DGS를 SLAM에 최초 도입, silhouette-guided differentiable rendering RGB-D. https://github.com/spla-tam/SplaTAM
- **MonoGS / "Gaussian Splatting SLAM"** (CVPR 2024 Highlight, Matsuki/Murai/Kelly/Davison, Imperial). 최초 monocular 3DGS SLAM, analytic Jacobian. https://github.com/muskie82/MonoGS
- **Photo-SLAM** (CVPR 2024, Huang & Yeung, HKUST). ORB-SLAM3 pose 사용 하이브리드 Gaussian 맵, mono/stereo/RGB-D 실시간. https://huajianup.github.io/research/Photo-SLAM/

**2025–2026 후속**
- **Splat-SLAM** (CVPR 2025 W, Sandström et al.). 최초 전역일관 frame-to-frame **RGB-only** 3DGS SLAM.
- **SplatMAP** (2025) — 온라인 dense monocular. Replica PSNR 36.86 보고 *[자체보고]*.
- **RGS-SLAM** (2026) — one-shot dense init, Replica ~925 FPS 렌더 *[2026 프리프린트]*.
- 대규모/멀티모달: **LSG-SLAM**(실외 stereo), **VIGS SLAM**(IMU 대규모), **LVI-GS**·**Gaussian-LIC2**(LiDAR-Inertial-Camera).
- 서베이: *3DGS in Robotics: A Survey* https://arxiv.org/html/2410.12262v2 · 큐레이션 https://github.com/3D-Vision-World/awesome-NeRF-and-3DGS-SLAM

### 1.4 세만틱 · 오픈보캐뷸러리 맵핑

**기반 (2023–2024)**
- **OpenScene** (CVPR 2023, Peng et al., ETH/Google). 3D point를 CLIP 임베딩과 co-embed, zero-shot 오픈보캐뷸러리 3D 이해. https://github.com/pengsongyou/openscene
- **VLMaps** (ICRA 2023, Huang/Mees/Zeng/Burgard, Freiburg/Google). 비전-언어 피처를 공간 맵에 융합, 언어조건 내비. https://github.com/vlmaps/vlmaps
- **OpenMask3D** (NeurIPS 2023). 오픈보캐뷸러리 3D **인스턴스** 분할.
- **ConceptGraphs** (ICRA 2024, Gu et al., Montréal/MIT/Toronto…). 오픈보캐뷸러리 **3D scene graph**, VLM/LLM 캡션+관계로 LLM 태스크 플래닝. https://concept-graphs.github.io/

**2025–2026 후속**
- **OpenVox** (2025) — 실시간 인스턴스 레벨 오픈보캐뷸러리 확률 voxel. https://arxiv.org/pdf/2502.16528
- **OpenMap** (ACM MM 2025), **OSMa-Bench**(조명 변화 하 오픈 세만틱 맵 평가), **DISC**·**IRIS-SLAM** *[2026 프리프린트]*.
- 서베이: *Semantic Mapping in Indoor Embodied AI* (2025) https://arxiv.org/pdf/2501.05750

### 1.5 SLAM 코드·ROS 성숙도 스냅샷

| 방법 | 코드 | 라이선스 | ROS/ROS2 |
|---|---|---|---|
| FAST-LIVO2 | hku-mars/FAST-LIVO2 | GPLv2 | ROS1 공식 / ROS2 커뮤니티 포트 |
| PIN-SLAM | PRBonn/PIN_SLAM | MIT류 | Python, ROS-friendly |
| KISS-ICP / KISS-SLAM | PRBonn | — | ROS-friendly |
| MASt3R-SLAM | rmurai0610/MASt3R-SLAM | MIT | 공식 없음 |
| SplaTAM / MonoGS | spla-tam, muskie82 | 연구 | 없음 |
| VLMaps | vlmaps/vlmaps | MIT | 내비 스택 포함 |
| ConceptGraphs | concept-graphs.github.io | MIT | 부분 (RGB-D 캡처) |

---

## 2. 경로계획 · 로컬플래너 · 모션제어

### 2.1 전역 플래너

**실전 주류 (Nav2 Smac suite)** — `ros-navigation/navigation2`, 유지보수 Steve Macenski 등.
- 한 플러그인에 **2D A***, **Hybrid-A***(Dubins/Reeds-Shepp, Ackermann/car/legged), **State Lattice**(차동/omni/Ackermann/legged kinematics, 사전계산 minimum-control set) 구현. CostmapDownsampler + Smoother 포함.
- **Kilted 개선:** `goal_heading_mode`, `coarse_search_resolution` 추가 → 한 번의 호출로 다중 목표 방향 계획.
- 그 외 Nav2 기본: **NavFn**(Dijkstra/A*), **Theta***(any-angle).
- 코드: https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner · 문서: https://docs.nav2.org/configuration/packages/configuring-smac-planner.html

**학술 개선(대부분 시뮬레이션 한정, 미productize):** Improved A*(16-이웃 하이브리드), Enhanced RRT*(~5.8% 짧은 경로/62.5% 적은 turning point *[자체보고]*), Improved B-RRT*, NPQ-RRT*. 서베이: https://pmc.ncbi.nlm.nih.gov/articles/PMC11861809/
> **시사점:** 배포 시스템에서는 Smac/Hybrid-A* 패밀리가 지배. RRT* 변형 문헌은 활발하나 대체로 시뮬레이션-only — 게재 스트림과 배포(Nav2) 사이 지속적 갭.

### 2.2 로컬 플래너 / 궤적 컨트롤러

앵커 레퍼런스: ROS 메인테이너 서베이 "From the Desks of ROS Maintainers" (Macenski et al., arXiv:2307.15236).

- **MPPI — Model Predictive Path Integral** (`nav2_mppi_controller`) — **현 SOTA 기본급 로컬 컨트롤러**. 매 사이클 Gaussian 노이즈 perturbation을 `batch_size`만큼 샘플→롤아웃→**critic** 비용함수로 가중. 차동/omni/Ackermann 지원. 모뎀급 4세대 i5에서 50+ Hz. **Kilted에서 Eigen 재구현으로 40–50% 성능 향상, ARM 지원**. "TEB·pure-path-tracking MPC의 후속". 코드: https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller
- **Regulated Pure Pursuit (RPP)** (Macenski/Singh/Martín/Ginés, *Autonomous Robots* 2023, arXiv:2305.20026). Pure-pursuit + 장애물 근처·협소공간 감속 regulation. 산업 서비스 로봇에 널리 배포. 안전중심·매우 강건.
- **DWB** (Nav2의 DWA 후속) — 동적윈도 속도 샘플 + critic. 역사적 기본값.
- **TEB — Timed Elastic Band** — MPC식 time-optimal, 궤적 변형. 협소 갭에서 경로효율 우수하나 제어주파수 낮음(~13Hz vs MPPI ~20Hz) *[단일연구]*.
- **RTEB — Resilient TEB** (2024/2025, Kulathunga et al., arXiv:2412.03174) — 미지환경 충돌회피 강화 TEB, ~17Hz *[자체보고]*.
- 기타: Graceful Controller, **Rotation Shim Controller**(주 컨트롤러 전 제자리 회전 — BARN식 트릭).
- 비교연구(2025): "Performance Analysis of DWB, MPPI, RPP, Rotate Shim for ROS2 AMR" / Wiley J. Field Robotics rob.22602.

### 2.3 동적 장애물 회피 · 반응형 (2025)

- **MPPI 로컬미니마 보완(핫 서브스레드):** **DRPA-MPPI**(Dynamic Repulsive Potential Augmented MPPI, Fuke/Endo/Honda/Ishigami, Keio Univ., IEEE CASE 2025, arXiv:2503.20134) — 예측된 entrapment 온라인 감지 → repulsive 비용항 자동 전환, 사전학습 불필요.
- **Control Barrier Function(CBF) 계열:** Dynamic Parabolic CBF(비홀로노믹, arXiv:2510.01402), Modulation+CBF 하이브리드("No Minima, No Collisions", arXiv:2502.14238), Velocity-Obstacle 기반 CBF.
- **학습 반응형(DRL):** 개선 TD3 mapless nav, transformer spatiotemporal-attention end-to-end 회피, 군중환경 DRL.
> **수렴 방향:** 고전 반응형(DWA, potential field)은 로컬미니마 취약 → 2025년 연구는 (a) MPPI+repulsive/potential 증강, (b) 최적화·RL 위 CBF 안전필터, (c) 최적화+학습 하이브리드로 수렴.

### 2.4 소셜 / 인간인지 내비게이션 (2024–2026)

핵심 서베이: "Social robot navigation: a review and benchmarking of learning-based methods" (*Frontiers in Robotics and AI* 2025). 큐레이션: https://github.com/Shuijing725/awesome-robot-social-navigation

- **Social-LLaVA** (2025, arXiv:2501.09024) — 소셜 컴플라이언트 내비용 VLM 파인튜닝, **SNEI 데이터셋**(40K VQA / 2K HRI). GPT-4V·Gemini 능가(인간평가) 보고, 모바일 로봇 온보드 배포.
- **VLM-Social-Nav** (IEEE RA-L, arXiv:2404.00210) — VLM 스코어링으로 소셜 비용항 생성, 기저 플래너 bias.
- **SoNIC** (2024, arXiv:2407.17460) — adaptive conformal inference + constrained RL. CrowdNav 96.93% 성공(이전 SOTA +~11.67%, 충돌 4.5× 감소) *[자체보고]*.
- **HEIGHT** (arXiv:2411.12150) — heterogeneous interaction graph transformer, 군중·협소.
- 데이터셋/벤치: **SCAND**(UT Austin, 텔레오퍼 소셜 데모, 기반 벤치), **SocialNav-SUB**(arXiv:2509.08757, VLM 장면이해 VQA), data-driven 소셜 메트릭(arXiv:2509.01251).

---

## 3. 학습기반 내비게이션 · Foundation Models

### 3.1 내비게이션 Foundation Models (end-to-end, goal-conditioned)

- **GNM** "A General Navigation Model to Drive Any Robot" (Shah et al., UC Berkeley, ICRA 2023). 이종 임베디먼트 교차학습 goal-conditioned(image-goal) 정책. **코드(MIT, ROS 통합):** https://github.com/robodhruv/visualnav-transformer
- **ViNT** "A Foundation Model for Visual Navigation" (Shah et al., CoRL 2023). Transformer goal-image 조건 내비 foundation, prompt-tuning 적응. (GNM과 동일 repo)
- **NoMaD** "Goal Masked Diffusion Policies" (Sridhar/Shah/Glossop/Levine, ICRA 2024). 단일 diffusion 정책이 goal 토큰 마스킹으로 탐색+목표내비 동시. (동일 repo)
- **Navigation World Models (NWM)** (Bar/Zhou/Darrell/LeCun, Meta/Berkeley/NYU, **CVPR 2025 Best Paper Honorable Mention**, arXiv:2412.03572). 제어가능 video world model(CDiT, ~1B), 후보 궤적 시뮬레이션·스코어로 계획. **코드:** https://github.com/facebookresearch/nwm
- **CityWalker** (Liu et al., NYU AI4CE, CVPR 2025, arXiv:2411.17820). ~2,000시간 도시보행 웹비디오에서 human-like 도시내비 모방학습. **코드(Apache-2.0):** https://github.com/ai4ce/CityWalker
- **LeLaN** (Hirose/Glossop et al., Berkeley/Toyota, CoRL 2024). 액션없는 egocentric 비디오에서 언어조건 객체내비 학습, ~4× 추론속도. https://learning-language-navigation.github.io/
- **MBRA / LogoNav** (2025). Model-Based ReAnnotation으로 passive 비디오 재라벨링 → 장기 goal-conditioned 정책. GNM/ViNT/NoMaD 직계. (venue 미확인)

### 3.2 RL · 모방학습 (ROS2 sim-to-real)

- **Sim-to-Real RL: Isaac Sim → Gazebo → 실 ROS2** (arXiv:2501.02902, 2025). 명시적 ROS2 통합 파이프라인.
- 최소센서 sim-to-real (MDPI Appl. Sci. 2025) — PPO/PPO-Mask, 휠 오도메트리+단일 2D LiDAR만으로 차동구동 배포, 재튜닝 없음.
- **QuasiNav**, **HDPlanner** (ICRA 2025, Clearpath Jackal 검증). quasimetric 임베딩 비대칭 비용 / 계층 attention 탐색.

### 3.3 Vision-Language Navigation (VLN) · VLA

- **NaVid** (Zhang et al., RSS 2024, arXiv:2402.15852). 최초 video 기반 VLM VLN, monocular RGB만으로 다음 액션 출력(맵·오도·depth 불필요). **코드:** https://github.com/jzhzhang/NaVid-VLN-CE
- **Uni-NaVid** (RSS 2025, arXiv:2412.06224). 다중 내비 태스크 통합 단일 video VLA. (동일 eval repo)
- **NaVILA** (Cheng/Ji et al., UCSD/USC/NVIDIA, **RSS 2025**, arXiv:2412.04453). **2단 VLA**: VLM이 중간레벨 언어명령("75cm 전진") 생성 → 실시간 RL **visual locomotion 정책**이 다족 로봇에서 실행. **VLN-CE-Isaac** 벤치(Unitree Go2/H1) 도입. **코드:** https://github.com/AnjieCheng/NaVILA · 벤치: https://github.com/yang-zj1026/VLN-CE-Isaac
- **StreamVLN** (InternRobotics/OpenRobotLab, **ICRA 2026**, arXiv:2507.05240). SlowFast 컨텍스트(빠른 sliding KV-cache + 느린 token-pruned 메모리)로 저지연 스트리밍 VLN. Unitree Go2 실배포, ~0.27s/4액션. **코드:** https://github.com/InternRobotics/StreamVLN
- **VLN-R1** (arXiv:2506.17221, 2025) — video-LLM VLN에 RL 파인튜닝(GRPO식).
- 기타 2025: CorrectNav(자기교정), Aux-Think(데이터효율), MonoDream(monocular panoramic), VL-Nav(neuro-symbolic).
- **[미확인 — 미래일자 프리프린트]:** "Ground Slow Move Fast"(2512.08186), Efficient-VLN(2512.10310, R2R 64.2% SR 주장), DualVLN, NavForesee, TIC-VLA 등 2025.12–2026 arXiv ID는 원문 확인 불가 — 리드로만 취급.

### 3.4 VLN/foundation 코드-성숙 출발점

- GNM/ViNT/NoMaD: https://github.com/robodhruv/visualnav-transformer (MIT, **ROS 완비**)
- NWM: https://github.com/facebookresearch/nwm · CityWalker: https://github.com/ai4ce/CityWalker
- NaVILA(+벤치): https://github.com/AnjieCheng/NaVILA · NaVid/Uni-NaVid: https://github.com/jzhzhang/NaVid-VLN-CE
- StreamVLN: https://github.com/InternRobotics/StreamVLN
- 큐레이션: https://github.com/jonyzhang2023/awesome-embodied-vla-va-vln

---

## 4. 플랫폼 · 실배포 (ROS2 Nav2 생태계)

### 4.1 Nav2 (Navigation2) 현황

ROS2 Navigation Framework. 유지보수 **Open Navigation LLC** (Steve Macenski 등), 후원 NVIDIA/AMD/Dexory. 배포: **Humble, Jazzy, Kilted, Rolling**. 가장 성숙한 오픈소스 내비 스택, ROS2 모바일 로봇 사실상 표준. 코드: https://github.com/ros-navigation/navigation2 · 문서: https://docs.nav2.org/

**핵심 컴포넌트(플러그인):**
- **컨트롤러:** MPPI, Regulated Pure Pursuit, DWB, Graceful (§2.2 참조)
- **플래너:** Smac(2D A*/Hybrid-A*/State Lattice), NavFn, Theta* (§2.1)
- **스무더:** Constrained Smoother, Velocity Smoother
- **Collision Monitor** (`nav2_collision_monitor`) — 코스트맵/플래너 우회, 원시 센서로 `cmd_vel` 필터링하는 독립 안전노드(긴급정지/감속)
- **Route Server** (`nav2_route`, Kilted) — 사전정의 **route-graph 계획**으로 전역플래너 보완/대체, 엣지/노드별 동작(문열기, 속도제한, 미래충돌검사) + 그래프 메타데이터. 결정론적 경로 필요한 산업/AMR에 핵심
- **Behavior Trees** (`nav2_behavior_tree`, BehaviorTree.CPP) — `bt_navigator`가 navigate-to-pose / through-poses / 복구 BT 실행
- **Docking Server** (`opennav_docking`, 2024.6~ 통합) — auto-dock/undock, AprilTag 결합 일반적
- **코스트맵 필터:** Keepout, Speed Limit 등

**릴리스 타임라인:**
- **Jazzy** (2024, Ubuntu Noble) — 도킹 통합
- **Kilted Kaiju** (2025.5) — MPPI Eigen 재구현(ARM 지원), Smac 다중방향 목표, Route Server 도입, **기본 `cmd_vel`이 `TwistStamped`로 변경**(stale 메시지 거부)
- **Rolling** — 개발 브랜치

> *주의: docs.nav2.org / ROS Discourse가 자동 fetch에 403 반환 → Kilted 세부는 스니펫 기반(여러 출처 일치하나 전체페이지 미확인).*

### 4.2 실배포 스택 (AMR/서비스/배송)

전형 아키텍처: Docker 컨테이너 ROS2 + LiDAR SLAM 맵핑 + Nav2 웨이포인트/회피 + **robot_localization** EKF/UKF 융합. **production AMR은 LiDAR SLAM이 가장 신뢰**, visual SLAM은 보조.

- 상용 AMR: KUKA AMR, **MiR**, Seegrid, Kollmorgen — 모두 **VDA5050**(개방 AGV/AMR↔플릿매니저 인터페이스, JSON-over-MQTT) 채택 가속(2025–2026).
- 레퍼런스 HW: **NVIDIA/Segway Nova Carter** — Nova Orin 센서스위트(스테레오4+피시아이4+IMU+2D RPLidar 2 + XT-32 3D LiDAR), Open Navigation Nav2 검증, Isaac AMR/ROS 레퍼런스.

### 4.3 로컬라이제이션 (production)

- **AMCL** (`nav2_amcl`) — 알려진 정적맵 입자필터, 실내 표준
- **slam_toolbox** (Macenski) — 2D lifelong 맵핑/로컬라이제이션, multi-session, Nav2 공식 지원. https://github.com/SteveMacenski/slam_toolbox
- **robot_localization** — EKF/UKF 범용 융합(휠오도+IMU+VO), `navsat_transform_node`로 **GPS 융합**. 실외는 dual-EKF(local odom + global map) 패턴 표준
- **Cartographer** — 일부 배포에서 여전히 사용

### 4.4 풀스택 · 플릿/산업 프레임워크

- **Open-RMF** (Open Robotics Middleware Framework) — 멀티플릿·멀티벤더 조율 레이어: traffic schedule DB, fleet adapter, 문/엘리베이터 통합, task dispatch, web UI. 배포 Humble~Rolling, amd64/aarch64. https://github.com/open-rmf/rmf · 책 https://osrf.github.io/ros2multirobotbook/
- **VDA5050** — 개방 AGV/AMR↔플릿매니저 표준(VDA/VDMA), Factsheet/Order/State/Actions. 혼합벤더 플릿. https://arxiv.org/pdf/2311.14615
- **NVIDIA Isaac ROS / Isaac AMR / Isaac Sim** — GPU 가속 인지(cuVSLAM, nvblox, AprilTag) + Nav2 통합, 포토리얼 디지털트윈 + ROS2 브리지.

### 4.5 배포 트렌드

- 플릿 상호운용성: **Open-RMF**(오케스트레이션) + **VDA5050**(벤더중립 프로토콜) 수렴. Nav2 + Open-RMF가 이종 플릿 문서화 패턴.
- 시뮬레이션-우선: **Gazebo**(개방·ROS 네이티브) + **NVIDIA Isaac Sim**(포토리얼·GPU·합성데이터·sim-to-real) 이중 생태계.
- HW 가속: ARM/GPU(Jetson Orin) 이동 — Nav2 MPPI ARM 재구현이 방증.

---

## 5. 벤치마크 · 데이터셋 · 시뮬레이터

### 5.1 내비게이션 벤치마크 · 챌린지

- **Habitat Challenge** (Meta AI) — PointNav(좌표도달·sim2real 예측성), ObjectNav(객체탐색·의미/상식), ImageNav(목표이미지). 메트릭: SR, **SPL**, SoftSPL, DTS. HM3D/MP3D 씬. https://aihabitat.org/
- **Habitat 3.0 / SIRO** (Meta, 2023.10~, arXiv:2310.13724) — 인간·아바타·로봇 공존 시뮬, Social Navigation/Rearrangement, HITL(VR). 데이터셋 **HSSD-200**(18,000+ 객체, 211 씬).
- **HM3D-OVON** (IROS 2024, arXiv:2409.14296) — 오픈보캐뷸러리 ObjectNav, 379 카테고리. https://github.com/naokiyokoyama/ovon
- **GOAT-Bench** (2024, arXiv:2404.06609) — 멀티모달 lifelong 내비(카테고리/언어/이미지 목표 시퀀스).
- **BARN Challenge** (GMU, Xuesu Xiao; ICRA) — Clearpath Jackal로 cluttered/협소 코스 무충돌 최속 주파. 시뮬 qualifier + 물리 finals.
  - ICRA 2024(3rd): 1위 LiCS-KI, 2위 MLDA_EEE, 3위 AIMS. 교훈논문 arXiv:2407.01862.
  - ICRA 2025(4th): 1위 RRSL(Michigan Tech), 2위 RobotiXX(GMU), 3위 UVA AMR. **동적 장애물 최초 도입**(동적충돌=감점, 정적충돌=완전실패).
  - BARN 2026 진행 중.
- **소셜:** SCAND, SocNavBench/SocialGym 2.0, SocialNav-SUB.
- **VLN-CE** — 주요 VLN 벤치(R2R, RxR 기반). 메트릭 NE/OS/SR/SPL/nDTW.
- **VLN-CE-Isaac** (NaVILA) — Isaac-Sim VLN, 다족 동역학(Go2/H1), RGB+depth.

### 5.2 시뮬레이터

- **NVIDIA Isaac Sim 5.0 / Isaac Lab 2.2** (SIGGRAPH 2025 GA, **오픈소스화**) — GPU 포토리얼(Omniverse/USD) + 학습 프레임워크. ViPlanner·NaVILA 기반. https://github.com/isaac-sim/IsaacSim · Isaac Lab arXiv:2511.04831
- **Gazebo** — Classic EOL(2025.1). 현대 계보: **Harmonic**(LTS, ROS2 Jazzy, ~2028), **Ionic**(ROS2 Kilted), **Jetty**(최신 LTS, ROS2 Rolling, ~2030). https://gazebosim.org/docs/latest/
- **Habitat-Sim** (Meta) — 10k+ FPS 포토리얼 임베디드 시뮬, Habitat Challenge/3.0 백본.
- **AI2-THOR / RoboTHOR** (Allen AI) — Unity 물리 인터랙티브, sim-to-real 페어드 환경(LoCoBot 배포).
- **iGibson / Gibson** (Stanford SVL) — 인터랙티브 실가정 씬, 장기 모바일 매니퓰레이션.

### 5.3 데이터셋

- **SLAM/오도:** KITTI/KITTI-360, EuRoC MAV, TUM-RGBD, Replica, ScanNet/ScanNet++, 7-Scenes, ETH3D-SLAM, Newer College, MulRan, HeLiPR, Apollo, **TartanAir**(CMU, 1037 시퀀스 멀티모달 스트레스 테스트).
- **자율주행/인지:** nuScenes, Waymo Open, A2D2, Oxford Radar, DSEC.
- **벤치마킹 프레임워크(2024–2025):** SLAM Hive(클라우드 스케일, arXiv:2406.17586), **VSLAM-LAB**(통합, arXiv:2504.04457).
- 큐레이션: https://github.com/youngguncho/awesome-slam-datasets

### 5.4 빅픽처 트렌드 (2025–2026)

- **Foundation/VLM/LLM ↔ 내비 수렴:** VLN(자연어 지시 3D 따라가기)이 중심. 서베이 "VLN Today and Tomorrow: A Survey in the Era of Foundation Models" arXiv:2407.07035.
- **VLA for navigation:** NaVILA식 "중간레벨 언어 액션 → RL locomotion"이 다족/휠 주류 레시피.
- **World model & embodied AI:** NWM 등. 서베이 arXiv:2407.06886, 2507.00917.
- **Sim-to-real이 조직화 문제:** 서베이 arXiv:2505.01458. 물리 동역학·렌더 격차가 지배적 장벽.
- 주도 기관: Meta AI(Habitat/SIRO), NVIDIA(Isaac/GR00T-adjacent VLA), Allen AI(THOR), Stanford SVL(iGibson/BEHAVIOR), 학계 VLN/VLA 그룹.
- 허브: https://github.com/HCPLab-SYSU/Embodied_AI_Paper_List

---

## 6. 종합 공개문제 (분야 합의)

1. **Sim-to-real 격차** — 물리 동역학(마찰·충돌·유체)·렌더(조명·노출) 불일치가 지배적 장벽. VLN sim→real 성공률 큰 폭 하락(맵 prior 있어도 ~55.9%→46.8%, 없으면 22.5%). (arXiv:2407.07035, 2505.01458)
2. **3D foundation model의 geometry 프런트엔드 대체** — MASt3R/VGGT가 dense SLAM 구동, 그러나 metric scale·투영 모호성(SL(4))·장기 시퀀스 확장 미해결.
3. **포토리얼+metric 맵핑 수렴** — 3DGS/NeRF SLAM이 실외·멀티모달·온로봇 실시간으로, deformable Gaussian 맵 내 루프클로저 난제.
4. **동적/군중/소셜 환경** — 정적 반응형 벤치는 "거의 해결"로 간주, 초점이 동적(BARN 2025 동적장애물)·소셜로. 표준 data-driven 소셜 메트릭 부재(arXiv:2509.01251).
5. **오픈보캐뷸러리 의미 → 실시간 온로봇** — 오프라인 scene graph(ConceptGraphs)에서 실시간 인스턴스 voxel/graph(OpenVox, DISC)로.
6. **실시간/엣지 추론** — VLA/VLM 내비는 Jetson급에서 ~10–20Hz, 제어 희망 50–100Hz와 격차. slow-fast KV cache(StreamVLN), one-shot init(RGS-SLAM), 직접 융합(FAST-LIVO2)이 온보드 타깃.
7. **장기 자율 · 맵 유지보수** — 변화환경 맵 최신화, re-localization, submap 그래프 sparsification.
8. **하이브리드 우세 & 게재-배포 갭** — 대회 우승·실배포는 고전+학습 하이브리드, 순수 end-to-end 아님. RRT*/A* 학술개선은 시뮬레이션-only 잔존.

---

## 7. 다음 단계로의 시사점 (아키텍처 설계 입력)

이 서베이가 가리키는 **권장 출발 아키텍처**(다음 단계에서 상세화):

- **베이스 스택:** ROS2 (Jazzy/Kilted) + **Nav2**. 검증된 모듈성·플러그인·플릿(Open-RMF)·안전(Collision Monitor)을 공짜로 확보.
- **상태추정:** 실내는 slam_toolbox(2D) + AMCL, 또는 3D가 필요하면 FAST-LIO2/KISS-ICP. robot_localization EKF로 융합. 실외는 GPS dual-EKF.
- **계획·제어:** Smac Hybrid-A*(전역) + **MPPI**(지역). 결정론 경로 필요시 Route Server. 안전은 Collision Monitor + (실험) CBF 필터.
- **연구 차별화 포인트(새 논문·기술 후보):** ① 3D foundation SLAM(MASt3R/VGGT)을 Nav2 코스트맵/로컬라이제이션에 실용 통합 — 현재 ROS 공식 부재라 빈틈. ② MPPI 로컬미니마 + CBF 안전을 결합한 ROS2-native 컨트롤러. ③ 오픈보캐뷸러리 실시간 세만틱 맵(OpenVox류)을 Nav2 코스트맵 레이어로. ④ VLA 중간레벨 언어명령을 Nav2 BT 액션으로 브리징.

> 이 4가지 "빈틈"은 모두 *실배포 성숙 스택(Nav2)* 과 *최신 연구(foundation/VLA/3DGS)* 사이의 통합 갭에 위치 — 새 기술·논문 개발의 유망 타깃.

---

### 부록: 신뢰도 종합 캐비엇
- 벤치마크 수치는 전부 자체보고, 독립 재현 아님. 교차 데이터셋 비교는 조건 의존(VSLAM-LAB: ORB-SLAM2가 KITTI에서 DROID/MASt3R 능가).
- 다수 2025.12–2026 arXiv ID(2512.x, 2601–2603.x)는 매우 최신/미래일자 프리프린트로 원문 확인 불가 — **[미확인]** 표기 항목은 리드로만 취급.
- docs.nav2.org·ROS Discourse·BARN(GMU) 페이지가 자동 fetch 403 반환 → 일부는 스니펫 기반(다중 출처 일치하나 전체페이지 미확인).
- SELM-SLAM3 87.84%, KISS-SLAM ROS 성숙도, BARN 2025 RRSL 우승 스택 내부, "Falcon" 소셜내비 등은 단일출처/미확인.
