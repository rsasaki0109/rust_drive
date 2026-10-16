# Path toward a broader Rust driving stack

The current engineering estimate is **about 20%**, compared with the scope, independent validation and operating experience of Autoware, Apollo and openpilot. Their operating domains differ. This is a subjective planning estimate, not a benchmark, safety score, percentage of source code or demonstrated parity.

The [camera-result freshness layer](camera-freshness.md) now separates source
and delivery clocks, expires stale results, excludes queued pre-fault images
and requires consecutive fresh acquisitions to recover. Independent checks
cover 46 synthetic sequences and four injected-control sequences on the same
eight saved indoor detector reports. All delivery delays and faults are
authored, so actual fault rates, automotive availability and object-identity
accuracy remain unmeasured. This improves failure behavior and keeps the
estimate at **about 20%**, with **30% unmet**.

The [recorded-image sequencer](camera-sequence.md) now runs detection and
tracking together with one reusable model plan, complete image preflight and
original acquisition times. Two runs of eight recorded indoor RGB frames
reproduce the separate tools and independent association/lifecycle checks.
This is real-input execution validation, without object-identity ground truth
or an automotive accuracy result. The estimate remains **about 20%**, with
**30% unmet**.

The [image-space tracker](image-tracking.md) now adds timestamped correspondence
hypotheses, motion prediction, global assignment and explicit fault/expiry
handling in an independent Rust library. Exhaustive synthetic checks and
admission of real detector reports verify implementation and the interface.
Real-sequence identity accuracy, metric object motion and vehicle integration
remain unverified, so the estimate stays **about 20%**, with **30% unmet**.

The [one-command offline camera tool](camera-analysis.md) now joins image
detection and vehicle consistency with bounded input/report handling and
complete, no-clobber publication. Repeated native inference preserves the
existing 78/138 matches and 31 false positives. This improves usability and
failure handling without improving accuracy or closing the remaining physical
validation gaps. The estimate remains **about 20%**, with **30% unmet**.

The [vehicle consistency adapter](yolox-vehicle-context.md) now keeps ambiguous
full-image vehicle labels instead of forcing a selection. On the same viewed
BDD frames it retains all 78 matched references and reduces false positives
from 33 to 31, raising precision from 70.27% to 71.56%. Two preceding failed
rules remain preserved. Recall, independent accuracy/calibration and sustained
map localization remain unresolved. The estimate stays **about 20%**, with
**30% unmet**.

The [bounded multi-view camera adapter](yolox-tiles-camera.md) improves viewed
BDD recall from 55/138 to 78/138 references while retaining every earlier S
match. Precision falls from 79.71% to 70.27% as false positives rise from 14 to
33. Overall recall and three class-recall gates still fail; descriptive
confidence reliability on viewed emitted detections supplies no held-out
calibration. This improvement leaves the estimate at **about 20%** and the
30% waypoint unmet.

The separate [immutable measured-depth map baseline](projective-fixed-map.md)
checks whether removing fusion and reference changes repairs continuous pose.
It scores 24/20/17 accurate updates out of 179 in room/desk2/RPY and loses all
three tracks. All 1,921 raw normal systems agree with independent reconstruction,
but this verifies implementation rather than physical accuracy or availability.
Real-motion localization and independent calibration remain unresolved; the
estimate stays **about 20%**, with **30% unmet**.

The optional [multi-reference RGB-D experiment](multireference-rgbd.md) adds
real image-residual estimation and independently reconstructed synthetic
matrices, but its viewed recorded accuracy regresses. Phase-aware fusion and
published-intrinsics comparisons also fail to improve both recordings. These
results strengthen diagnosis and preserve failed trials without raising the
estimate above **about 20%**.

The subsequent [joint keyframe pose comparison](joint-keyframe-depth.md)
corrects synthetic reference errors but regresses on viewed recorded motion:
room 72/179 and desk2 51/179 accurate, both with tracking loss and desk2 maximum
accepted position error 1.706278 m. Independent matrix/state verification
does not turn this into a physical-accuracy success. It remains a NumPy
research tool, without replacing the Rust default or increasing the
**about-20%** engineering estimate.

The [marginal information-retention comparison](marginalized-keyframe-depth.md)
then preserves derived depth factors across keyframe eviction. Both viewed
recordings accept all 179 updates without expiry; complete accuracy still
fails at 70/179 room and 74/179 desk2. Independent raw matrices, quaternion
transport, QR elimination and atomicity checks verify the implementation,
without supplying calibrated uncertainty, fresh environments or vehicle
integration. The engineering estimate remains **about 20%**.

The subsequent [subpixel depth-factor comparison](subpixel-keyframe-depth.md)
keeps the same information-retention engine and tests continuous inverse-depth
interpolation with projective gradients. Both viewed records accept 179/179,
but accuracy is 69/179 room and 74/179 desk2, with larger maximum errors than
the preceding 70/179 and 74/179 retained control. Independent reconstruction
of 8,336 systems and the preserved failed first trial improve auditability,
without establishing better physical accuracy, calibrated uncertainty or a
new operating domain. The estimate remains **about 20%**; the 30% and 50%
waypoints remain unmet.

The [symmetric measured-normal comparison](symmetric-keyframe-depth.md)
then keeps the retained-information engine and fixed-pose sampling while
using both normals and their rotation derivatives. It improves room from
70 to 72/179 accurate updates and keeps desk2 at 74/179, accepting all updates
and reducing maximum position errors to 0.172677 m / 0.239761 m. Independent
reconstruction of 7,916 systems and the preceding prior-influence diagnostic
verify the stated implementation. Both complete accuracy protocols still fail;
these viewed-data gains do not establish confidence calibration, a new physical
environment or vehicle integration. The estimate remains **about 20%**.

The subsequent [Tukey weighting comparison](robust-keyframe-depth.md) retains
the symmetric residual but changes influence and pair admission. It accepts
all 179 updates in each viewed record, scoring 71/179 room and 74/179 desk2
accurate. Desk2's maximum position error shrinks from 0.239761 to 0.198242 m,
while room worsens from 0.172677 to 0.262596 m. Independent reconstruction of
7,888 systems and residual/normal-disagreement diagnostics preserve this
tradeoff. Both complete accuracy protocols fail; calibrated confidence,
new-environment validation and vehicle integration remain unverified.
The estimate stays **about 20%**, with 30% and 50% still unmet.

The preceding estimate of about 8% reflected authored planar road behavior, observed-motion prediction, mapped infrastructure rules, fault recovery and optional inclined native LiDAR. The new increment adds operational measured-ground separation on physical native query surfaces, an explicit rectangular research body with independently reconstructed swept static-box clearance and actual native overlap rejection, and a bounded attributed external OSM importer. Those functions are integrated: genuine imported road coordinates drive native sensor acquisition, localization, planning, control and complete sensor-only replay in nine verified episodes. The new ground/body matrix has 60 healthy episodes and six separate confidence-stop cases. [Measured ground/body evidence](ground-lidar.md); [external-map evidence and repaired sharp-turn failure](osm-import.md).

The preceding 15% increment reflects those executable capabilities and independent checks. It did not increase for GIF quality or test counts. The research box is authored calibration, not a measured production vehicle; moving actors retain circular acceptance, native ego motion is planar, and Rapier overlap witnesses do not supply chassis contact response. Ground fitting assumes broad, near-flat support, can remove near-plane obstacle surfaces, and can falsely block on sparse/narrow ground. External OSM supplies historical geographic centerlines with explicit simulated widths, without lane topology; bounded unconditional node-via turn rules have since been implemented ([scope](turn-restrictions.md)). Explicit chassis-reference prediction and course-based steering repair the tested sharper native branch; its original failure remains historical evidence, and broader sharp-route feasibility remains unverified.

The new increment adds actual measured-data evaluation, bounded classical XYZ terrain/components, operational local scan-to-map EKF corrections and five-second GNSS-denied reference/native driving with independent expiry/failure controls. It **does not reach the requested 30%**. The frozen terrain baseline has calibration F1 0.7041 but held-out F1 0.2592, including six zero-F1 sites. Four apartment pose successes are warm-initialized imposed transforms of measured geometry; natural-pair alignment is rejected, and they do not establish real-motion pose accuracy or covariance calibration. An initial sparse native terrain/object acquisition conservatively blocks on residual ground and misses actor clusters. Existing dense acquisition and a declared research sensing height subsequently repair one native lead-stop case, with actual actor membership and complete replay checked independently. The failed sparse capture remains replayable. These failures remain evidence, rather than being removed or retuned against held-out labels. [Measured-data results](datasets.md); [bounded localization](map-localization.md); [experimental native objects](terrain-objects.md).

The latest increment adds density-aware terrain processing with a verified native lead-stop run, bounded local SE(3) matching against actual recorded depth/mocap, and optional Rust-native CPU learned image detection. The new environment terrain gate still fails (Autzen F1 0.3571); the original natural-pair protocol accepts 7/11 later temporal pairs in one room and rejects four, with markedly overconfident conditional covariance. All failed stages remain reproducible. These working but narrow additions support an estimate of about 20%; **the requested 30% remains unmet**. [Adaptive terrain](adaptive-ground.md); [recorded motion and covariance failures](recorded-rgbd.md); [optional inference](../integrations/onnx/README.md).

The subsequent [urban-camera diagnostic](road-camera-evaluation.md) actually
executes six real photographs with independent annotations: evaluator-held-out
precision 0.778 and recall 0.560. Its COCO validation images may overlap upstream
model selection, and traffic lights and small road users are missed. It supplies
reproducible diagnostics, without proving independent driving-domain accuracy.
Additional recorded-depth odometry and fixed-first-cloud localization preserve
sensor-only initialization, physical-reference gaps and rejected map fits. Their
engineering covariance allowance is explicitly provisional, rather than a
calibrated confidence guarantee. These additions do not increase the overall
estimate or close the remaining independent-environment and calibration gates.

Measured keyframe localization additionally replaces its reference with accepted
recorded clouds and latches loss on expired accepted poses, including malformed
acquisitions. It repairs the viewed short fixed-map rejection case, but the
preregistered longer temporal trial has only **13/35 root-accurate updates** and
two ambiguity rejections. The maximum accumulated error is 0.170314 m /
0.163528 rad. Its original sources and failure remain immutable; subsequent
guard fixes and repeats are explicitly viewed regressions. This does not raise
the estimate, establish automotive localization, or close the 30% waypoint.
[Keyframe protocol and evidence](recorded-keyframes.md).

General reliable terrain, measured vehicle/extrinsics, automotive camera perception, SLAM, lane topology, broader interaction/control and real-vehicle interfaces remain unresolved. The broader 20% calibration/contact goals also remain open. The estimate recognizes working local localization and reproducible measurement capability, without calling the calibration/contact waypoint or the 30% waypoint complete.

The fixed-root fused-submap implementation adds atomic bounded voxel fusion and independent reconstruction of every map generation. The first desk trial still passes root accuracy on only **8/35** updates; the separate office/camera trial scores **5/35** and latches loss after accepted-pose expiry. The separately frozen original BDD dashcam diagnostic finds **35/138** reference objects (25.4% recall), with no correct bicycle, motorcycle, truck or traffic-light detection. Working implementations and broader independent measurements do not repair these failures. The estimate remains **about 20%**, and the **30% waypoint remains unmet**. [Mapping evidence](recorded-submaps.md); [automotive image evidence and data terms](bdd-road-evaluation.md).

## Evidence used for assessment

Recorded RGB-D visual motion now adds original Rust image features, measured
depth correspondences and bounded robust fitting. Independent pixel/descriptor
reconstruction and a separate SVD fitter reproduce the operational estimates
and rejections. The viewed desk result improves to **24/35** accurate updates,
but the office configuration still scores only **9/35**. Neither completeness
nor broad accuracy is established. The separate first frozen sitting recording
scores **31/35**, retaining three consensus rejections and one repeated image.
This path supplies no vehicle controls.
The estimate remains **about 20%**; the 30% waypoint and requested 50% goal
remain unmet. [Full visual-motion evidence](recorded-visual-odometry.md).

The additive [pixel refinement](recorded-reprojection.md) repairs the viewed
office accuracy failure to **32/35**, with maximum accepted position error
**0.042368 m**. This is a useful measured localization improvement. Repeated
images and coarse failures remain in all denominators; the method remains
offline indoor odometry without vehicle calibration, calibrated uncertainty,
automotive perception accuracy or driving integration. The estimate therefore
remains **about 20%**; neither the 30% waypoint nor the 50% goal is complete.
The separate first Freiburg 2 attempt fails strict whole-source label validation
on conflicting duplicate timestamps. Subsequent viewed runs retain 24 sensor
updates, but all 35 accuracy references remain unavailable; this supplies no
additional independent accuracy evidence.

The separately frozen [metadata-qualified room trial](qualified-recorded-motion.md)
passes all 35 updates in a 1.1684-second indoor interval with unchanged gates.
Whole-source timestamp checks now precede image decoding, and the independent
pixel/refinement audit and exact viewed reproduction retain the first sources.
This is new short-sequence evidence, without automotive calibration, uncertainty,
long-duration mapping or driving fusion. The estimate remains **about 20%**;
the 30% waypoint and requested 50% goal remain unmet.

The [same-room continuous extension](temporal-recorded-motion.md) retains the
passing prefix but scores only **52/179** root-accurate updates over 5.973 seconds.
Orientation error exceeds the unchanged gate and insufficient measured matches
lead to latched tracking loss. This failure limits the preceding short result;
one origin, all failures and exact prefix reproduction are retained. The
estimate remains **about 20%**, with the 30% and 50% goals unmet.

The viewed [depth-supported matching variant](depth-supported-matching.md)
raises accepted updates by one but lowers accurate updates from 52/179 to
49/179. Its independent audit preserves that regression. It remains opt-in;
this result does not justify a maturity increase. The estimate stays **about 20%**.

| Area | Current evidence | Evidence needed for a credible 50% planning milestone |
|---|---|---|
| Sensing / localization | Native XYZ LiDAR, optional measured AABBs/terrain; failed held-out airborne PMF baseline; local fixed-map matching with bounded GNSS-denied native driving; noisy GNSS/wheel/gyro and acquisition-time reprojection | Reliable held-out automotive perception and real-motion map matching; measured extrinsics and camera observations; synchronization, covariance calibration and broader degraded-sensing measurements |
| Maps / behavior | Bounded attributed OSM centerline import with unconditional motorcar node-via no/only rules plus authored graphs/closures; infrastructure signal feed, stop-sign holds and basic fixed-route yielding | Lane topology, conditional/way-via restrictions and more external layouts; general intersections/lane changes and interaction evaluation |
| Prediction / planning / control | Current-time observed-motion forecasts, lateral lattice, reachable speed profiles, uncommitted approach bounds and native ego dynamics | Calibrated footprints and actuator models; broader interaction forecasts; optimized planning/control; independent feasibility and difficult-world regressions |
| Simulation / validation | CPU reference/native RNE runs; physical road/obstacle query surfaces; independent static-box sweeps, actual force-free overlap negatives, circular traffic scoring and complete replay | Physical 3D sensor scenes, contact/collision callbacks, multiple environments, varied layouts/traffic, reproducible failures and scenario coverage |
| Engineering / integration | Locked Rust workspaces, three-platform CI, English docs, executable demos | Stable adapter contracts, trace migrations, resource/deadline benchmarks, failure containment, external reproducibility and contributor workflows |
| Deployment / safety evidence | Simulation only | Explicit operating domains and hazard analyses; qualified review before any hardware experiments; no simulator-only road-safety claim |

## Capability gates, not promised release dates

| Planning waypoint | Required work and acceptance |
|---|---|
| Around 10% | Broader authored road behavior: stop signs, signal faults, basic yielding/intersections; fixed rule/collision gates; varied maps, speeds and sensor timing; documented failures |
| Around 15% | Measured ground processing on physical native query surfaces, an explicit rectangular research body with independent swept clearance and native overlap rejection, attributed external-map import, and an integrated sensor-only native run/replay; preserve known failures and preceding acceptance floors |
| Around 20% | Broader sensor/support geometry and external road layouts; measured vehicle/extrinsics calibration; broader native sharp turns; independent contact/rule outcomes and reproducible failure coverage |
| Around 30% | Real-data perception and map-localization baselines; optional learned perception with licensed/versioned models; held-out accuracy and calibration checks; bounded GNSS-denied regressions |
| Around 40% | Lane topology/changes, priority reasoning and interaction-aware prediction; optimized longitudinal/lateral control; varied-traffic and actuation-fault acceptance |
| Around 50% | Those capabilities integrated into repeatable end-to-end runs in multiple environments; independently checked resource/latency and failure behavior; stable public interfaces, limitations and external reproduction evidence |

A waypoint is not reached simply by implementing one row or generating more scenarios. Reassess the whole stack against measured capability, validation depth and practical use. Keep unresolved failures in the report, preserve preceding acceptance gates, and publish the evidence that justifies an estimate change. Reaching the 50% planning milestone would still not establish mature real-vehicle deployment or safety certification.

The chronological implementation roadmap remains in [roadmap.md](roadmap.md); executable boundaries are in [capabilities.md](capabilities.md). Signal-control commands and evidence are in [traffic-signals.md](traffic-signals.md); stop-sign behavior is in [stop-signs.md](stop-signs.md); basic priority yielding is in [intersections.md](intersections.md).


The [pairwise image-patch tracker](pyramidal-recorded-tracking.md) supplies an
original bounded Rust implementation and independently reconstructed pixel
measurements, but its viewed continuous result regresses to **6/179** accepted
and accurate updates. Repeated-texture controls accept incorrect endpoints;
rotation controls expose substantial accepted errors. First outcomes, source
archives, auditor-only repairs and every failure remain preserved. This does
not justify a maturity increase: the estimate stays **about 20%**, with 30%
and 50% unmet. The separate-recording acquisition blocker was subsequently
resolved for official FR1 desk2; the historical proxy-denial journal remains
preserved, and does not describe current source availability.


The separately preregistered [official desk2 recording](independent-recorded-motion.md)
now has an acquired archive, whole-source timestamp qualification and a source
freeze verified before recorded pixels. Its original continuous trial retains
all 179 updates over 5.974763 s: **16 are accepted and root-accurate, 163 are
rejected**, and accepted-pose expiry latches tracking loss at depth index 127.
Maximum accepted errors of 0.087690 m / 0.092509 rad satisfy the unchanged local
gates, but limited conditional accuracy does not satisfy continuous availability.
The original formal checker fails on an inherited calibration-schema expectation;
a separate additive audit repair now passes integrity and independently scores
all 179 outcomes, without changing original sources or results. Twelve
calibration/inventory, 44 source and 73 sensor/state corruptions are rejected.
Integrity success and failed physical acceptance are distinct outcomes.

Desk2 is a second recording of the same four desks, not a new physical
room or automotive environment. It is now viewed regression data. This failed
trial broadens measurement evidence but does not justify a maturity increase:
**about 20%** remains a subjective assessment; the **30% and 50% waypoints
remain unmet**. Automotive perception accuracy, sustained real-motion localization,
measured extrinsics, calibrated uncertainty and vehicle-pipeline integration
remain material gates. No proposed multiscale front end is claimed implemented
or validated by this trial.


The [completed desk2 viewed regression suite](../assets/recorded-independent/desk2-regression/suite.json)
reproduces every first-trial non-timing outcome, with successful qualification,
freeze preparation and independent integrity audit while preserving evaluator
exit 1. Separate output-admission controls and a historical CI build-target
repair improve reproducibility without changing measured motion accuracy.
They do not increase the subjective estimate above **about 20%** or close the
**30%** waypoint.

The additive [multiscale feature diagnostic](multiscale-features.md) improves
viewed independent adjacent-pair accuracy from 138 to 165 of 179 room pairs and
124 to 154 of 179 desk2 pairs. Maximum fitted translation and rotation errors
increase in both recordings, and the analytic rotation control retains seven
false matches. Final-source independent mathematical audits verify all 179 pairs
in each recording and reject 20 report corruptions each. Output-admission guards
are separate evidence, with no estimator retuning. This frontend has no
continuous root trajectory, tracking-permission state or vehicle integration;
the original room 52/179 and desk2 16/179 continuous failures remain unchanged.
Viewed pair support and byte-identical repeats do not close calibrated
uncertainty, sustained localization or automotive perception gates. The
subjective estimate remains **about 20%**, with **30% and 50% unmet**.

The first [continuous multiscale comparison](multiscale-temporal.md) tests that
integration boundary with unchanged root gates and all 179 updates retained.
The room branch accepts 92 updates versus native 58 but scores only 49 accurate
versus 52; desk2 accepts 19 versus 16 and scores 15 versus 16 accurate. All
branches lose tracking, and maximum accepted root errors increase. This
preserved viewed regression demonstrates that better adjacent-pair support
does not establish improved accumulated root accuracy. Full independent audits
verify all 180 frames / 179 updates, rejecting 15 corruptions on room and 17 on
desk2; complete repeat validation reproduces both source-bound non-timing reports.
The failed estimator
outcome is separate from integrity success. The 34 extra accepted room updates
contribute no accurate roots, while both recordings lose accuracy on some
previously accurate common fits. Passing 371 core and 94 optional Rust tests
does not alter that measured regression. The estimate remains **about 20%**, with
the **30% and 50% waypoints unmet**.

The [registered-grid camera interpretation](registered-grid-camera.md) follows
official guidance for already registered/pre-scaled TUM images, using explicitly
uncalibrated default intrinsics rather than claiming measured calibration.
Its first viewed continuous comparison improves room native accuracy from 52
to 56/179, room multiscale from 49 to 54/179, and desk2 multiscale from 15 to
17/179; desk2 native remains 16/179. Accepted counts and loss indices are
unchanged and all branches lose tracking. Complete independent audits and exact
non-timing regression replay confirm the failed physical outcomes. This does
not establish sustained localization or calibrated uncertainty.

The separate [YOLOX-S comparison](yolox-s-camera.md) executes the fixed official
source/model profile on eight already viewed automotive images. Matches rise
from 35 to 55/138 (34 retained, 21 added, one lost), but recall **39.86%** fails
the 60% gate; person and bicycle recall remain below 50%, and no motorcycle or
traffic light is matched. Sixteen actual CPU inference runs pass and repeated
detections agree; quality acceptance fails. First-run inference averages
0.606 s/image on this host, without a real-time claim. Complete replay verifies
all eight detection sets within absolute tolerance 1e-5 and exactly reproduces
match IDs/counts and failed gates; the wrapper still exits 1 for quality.
Neither increment supplies a new environment, vehicle calibration or
camera-derived driving controls. The estimate remains
**about 20% subjectively**, with **30% and 50% unmet**.

The [dense-depth and OpenCV diagnostics](projective-depth-diagnostics.md)
retain six viewed-data development variants and a pinned OSS reference.
Projective depth fusion improves desk2 to **161/179** accurate roots, but room
scores **64/179**; both still fail complete accuracy. The reference scores
**83/179** and **15/179**, with all updates accepted. Independent root scoring,
18 corruption rejections and exact reference replay distinguish integrity
from measured accuracy. The exploratory algorithms remain outside the Rust
driving pipeline and provide no held-out automotive or calibrated uncertainty
result. The subjective estimate remains **about 20%**; the **30% waypoint is
blocked by unresolved accuracy and integration gates**, rather than completed
by extra implementations or successful test execution.

The subsequent [Rust projective-depth implementation](projective-depth-rust.md)
ports the sensor-only fusion variant into an independent bounded library and
CLI. It reproduces the viewed **64/179 room and 161/179 desk2** outcomes;
independent NumPy checks verify 6,861 normal steps and the pose/reference/clock
state, with analytic motion, degeneracy and corruption controls. This is an
executable Rust improvement, without additional physical accuracy or driving
integration. Both complete recorded protocols still fail, so the subjective
estimate remains **about 20%** and the **30% waypoint remains unmet**.

The [explicit-clock depth supervisor](projective-depth-supervision.md) adds
idle-tick expiry, fault-epoch quarantine and qualified short-fault recovery.
Separate clean estimator sequences check that failed/quarantined depth cannot
change later map/reference/pose estimates. This improves temporal failure
handling, without repairing room/desk2 root accuracy or supplying a live
vehicle integration. The subjective estimate remains **about 20%**, with the
**30% waypoint unmet**.

The [raw-depth reconstruction audit](projective-depth-raw-audit.md) closes the
earlier explicit correspondence-audit gap: independent PNG-derived geometry
reproduces all 6,861 iteration equations, 358 final equations and the prescribed
fused models, with exact discrete counts. It detects jointly rescaled H/g that
still pass a solve-only check. This strengthens input-to-report integrity;
room/desk2 physical root accuracy, calibration and vehicle integration remain
unchanged. The subjective estimate stays **about 20%**, with **30% unmet**.

The [local-plane normal experiment](patch-keyframe-depth.md) reduces normal
error in a fixed noisy-plane fixture using 5×5 measured patches. In the two
viewed recordings, it accepts only **142/179 room and 120/179 desk2** updates
and both branches latch loss; physical accuracy is **77/179 and 75/179**.
This fails both complete protocols. It provides an independently checked
normal frontend and failure evidence, rather than broader localization or
vehicle capability. The Rust default stays unchanged and the subjective
assessment remains **about 20%** against mature OSS; the **30% and 50% goals
remain unmet**.

The [selective depth-level schedule](selective-keyframe-depth.md) repairs the
preceding patch-normal branch's viewed tracking regression: both records now
accept **179/179** updates without terminal loss. Coarse proposals can be
omitted only with a recorded guard failure, while finest-level, final-pose and
Schur requirements remain strict. Physical accuracy is **77/179 room and
76/179 desk2**, so complete accuracy remains unmet.
This is an optional NumPy improvement on already viewed indoor data, with
unchanged Rust defaults and no new vehicle integration or independent
environment evidence. The subjective maturity assessment remains **about 20%**;
the **30% and 50% goals remain unmet**.

The [reciprocal projected-pixel comparison](reciprocal-keyframe-depth.md)
adds exact return-to-source pixel selection and independent camera-matrix
verification. On already viewed recordings, room accuracy rises from **77 to
88/179**, with three condition rejections, while desk2 falls from **76 to
75/179** and its maximum errors increase. Both tracks avoid terminal loss;
both complete physical protocols still fail. A short known-noise control and
fixed-pose diagnostics do not establish the cause of real drift or a new
environment result. The Rust default stays unchanged, and the subjective
assessment remains **about 20%**; **30% and 50% remain unmet**.

The [bounded continuation diagnostic](depth-convergence.md) adds reproducible
local convergence checks and rejects six classes of corrupted reports. Ten
additional finest-level updates converge in **0/10** saved fits. Changing
correspondence populations prevent interpreting raw RMS changes as a common
objective or physical-accuracy gain. No new trajectory, environment or vehicle
integration is evaluated; iteration caps and the subjective **about-20%**
estimate remain unchanged, with **30% and 50% unmet**.

The [frozen-correspondence descent control](descent-keyframe-depth.md) adds a
bounded operational step check, a separately reconstructed nonlinear objective
and six search-report corruption controls. This verifies a local update
property over fixed measurements and the transported prior. All **7,427**
recorded tentative steps use full width; both entire state histories and
physical scores match the reciprocal control at **88/179 room** and **75/179
desk2** accurate. Both complete protocols remain unmet. It does not
establish calibrated confidence, convergence after correspondence changes,
new-environment accuracy or vehicle integration. The subjective assessment
remains **about 20%**, with **30% and 50% unmet**.

The [pixel correspondence diagnostic](depth-correspondence.md) reconstructs
960 raw-pair transitions from 100 saved continuation steps. Fixed-pair cost
decreases in all steps; fresh correspondence populations yield higher costs
than the starting populations in **49/100**. Exact pixel IDs and independent
cost partitioning identify this local effect, without establishing which
matches are physically correct or explaining absolute trajectory error. It
fits no new poses and adds no physical environment or vehicle integration.
The subjective estimate remains **about 20%**; **30% and 50% remain unmet**.

The [fixed-pair local refinement comparison](fixed-depth-refinement.md) adds a
bounded nonlinear solver with independently reconstructed matrix derivatives
and step certificates. All ten saved windows reach fixed-population convergence
in 24 corrections, while fresh-population convergence remains **0/10**. The
predeclared progression condition fails; the solver is not integrated into the
tracker and supplies no new physical trajectory accuracy. The subjective
estimate remains **about 20%**, with **30% and 50% unmet**.

The [alternating local refinement comparison](alternating-depth-refinement.md)
adds a bounded outer correspondence loop and independently checked batch
anchors, budgets and termination. It repairs the rendered same-start fresh
convergence case, while all ten viewed recorded windows exhaust ten corrections
with **0/10** fresh convergence. The progression rule remains unmet; no new
continuous physical trajectory or tracker integration is claimed. The
subjective estimate stays **about 20%**, with **30% and 50% unmet**.

The [thirty-correction local diagnostic](budget-depth-refinement.md) tests
whether the preceding ten-correction allowance explains nonconvergence.
It exactly reproduces the original ten updates and retains every threshold,
but fresh convergence remains **0/10** at twenty and thirty corrections.
Independently checked pixel turnover and cost partitioning narrow the observed
local behavior without proving physical drift causation. The diagnostic adds
no new trajectory accuracy, physical domain or vehicle integration, and the
larger allowance is not adopted by tracking. The subjective assessment remains
**about 20%** versus mature driving OSS; **30% and 50% remain unmet**.

The [continuous fixed-source residual comparison](continuous-depth-refinement.md)
adds a constant-observation inverse-depth objective with independent polynomial
sampling, complete search replay and twelve prior-aware gradient axes. Its
rendered warm start converges, but a perturbed start fails and all ten viewed
real windows stop through support/search guards. A post-hoc replay identifies
the strict domain's sensitivity to losing only 1–4 source occurrences at the
smallest candidate step, without proving physical outliers or drift causation.
There is no new trajectory accuracy, operational integration or physical domain
result. Maturity remains **about 20%** against mature driving OSS; **30% and 50%
remain unmet**.

The [initial-margin continuous comparison](margin-depth-refinement.md) preserves
the solver and objective while initially qualifying a nine-point image stencil.
Independent controls verify selection and every new/control system, but real
convergence stays **0/10**. Combined stencil/minimum-support admission retains
only **10.78%** of source occurrences, replacing many search failures with
missing-support failures. This failed availability tradeoff supplies no new
trajectory accuracy, physical domain or driving integration. The subjective
estimate stays **about 20%** against mature driving OSS; **30% and 50% remain
unmet**.

The [sampling-domain comparison](sampling-depth-refinement.md) jointly limits
the initial stencil and repeated candidate checks to interpolation support,
while preserving center geometric matching. It retains 53.74% of center-qualified
source occurrences and reduces local guard stops from 10/10 to 5/10, but all
ten attempts still lack continuous and original-discrete fresh convergence.
The 54 tentative local corrections and independent system checks are not new
physical accuracy, accepted tracker updates or a driving integration result.
The subjective maturity estimate stays **about 20%** versus mature driving OSS;
**30% and 50% remain unmet**.

The [fixed-observation budget extension](long-depth-refinement.md) preserves
sources and weights across a maximum thirty corrections and exactly reproduces
all ten earlier cap-10 reports. Two desk2 windows reach continuous-objective
convergence at corrections 16 and 19; eight cases still end through guards,
and original-discrete fresh convergence remains **0/10**. Independent long,
control and checkpoint reconstruction verifies this partial local gain. It
does not supply new physical trajectory accuracy, a new environment or vehicle
integration. The subjective estimate remains **about 20%** versus mature driving
OSS; **30% and 50% remain unmet**.

The [bounded-search diagnosis](depth-search-diagnostic.md) finds smaller
decreasing steps at all four archived room failures and verifies their local
directional derivatives. Its fixed signed sweep performs no optimization
continuation or physical trajectory scoring. This narrows a local numerical
limitation while leaving the latest 2/10 continuous convergence and missing
automotive integration unchanged. The subjective estimate stays **about 20%**
versus mature driving OSS; **30% and 50% remain unmet**.

The [deeper bounded-search comparison](deep-search-depth-refinement.md) extends
only the candidate range, preserving all old trace prefixes and unchanged
terminal reports. Search stops fall from four to one, but three windows spend
their thirty-correction budget and convergence remains **2/10**. The two
slack-permitted tiny cost increases and failed progression rule are retained.
There is no new trajectory accuracy or vehicle integration. The subjective
estimate stays **about 20%** against mature driving OSS; **30% and 50% remain
unmet**.

The [conditional damped-direction comparison](damped-depth-refinement.md)
selects 32 alternatives while preserving undamped full-proposal convergence.
All three budget stops remain and their final objectives regress slightly;
continuous convergence stays **2/10**. The failed progression result and nine
tiny GN cost increases are retained. Numerical direction checks supply no new
physical trajectory accuracy or vehicle integration. The subjective estimate
stays **about 20%** versus mature driving OSS; **30% and 50% remain unmet**.

The [cell-transition slope diagnosis](depth-cell-slopes.md) attributes local
slope reversals at six saved budget states to a single interpolation-cell
change per interval. Independent calculations verify all probes, while the
same viewed windows, observations and physical scoring limits remain. A
directional barrier supplies neither multidimensional convergence nor a new
vehicle capability. Latest convergence stays **2/10**, and the subjective
maturity estimate stays **about 20%** versus mature driving OSS; **30% and 50%
remain unmet**.

The [C1 cubic comparison](cubic-depth-refinement.md) verifies continuous first
derivatives and an independent Hermite oracle, but complete-stencil support
loss reduces convergence from 2/10 to 0/10. Its separately frozen margin follow-up
reaches 1/10, including the previously stalled room 240 window, while retaining
only 49.94% of source occurrences. The failed availability/progression tradeoff
and absence of new physical trajectory scores remain explicit. This leaves
the subjective estimate **about 20%** versus mature driving OSS; **30% and 50%
remain unmet**.

The [measured-normal four-corner Hermite variant](normal-hermite-depth-refinement.md)
retains 100% of initial observations and converges on 4/10 local windows versus
the bilinear control's 2/10. Three room gains accompany a desk2 regression;
freshly associated convergence stays 0/10 and room fresh RMS regresses. The
independent audit and frozen failed progression result improve local evidence,
without establishing new physical trajectory accuracy or vehicle capability.
Subjective maturity stays **about 20%** versus mature driving OSS; **30% and 50%
remain unmet**.

The [local measured-normal support experiment](local-normal-depth-refinement.md)
extends a previously blocked room sampling domain with fully measured 3×3 plane
normals, independently checked by SVD. All initial observations and nine entire
control reports remain equal. The changed room window still stops, with a
smaller remaining full proposal but a worse fresh residual; convergence stays
**4/10 own / 0/10 fresh**. This supplies no new physical trajectory accuracy or
vehicle capability. Subjective maturity stays **about 20%** versus mature
driving OSS; **30% and 50% remain unmet**.

The [depth-derived C1 experiment](depth-slope-refinement.md) retains its first
3/10 regression and a separately frozen one-sided follow-up. The follow-up
keeps all initial sources and converges on 5/10 windows versus the previous
normal-based 4/10, with 97 full applied steps and no search stop. Four selected
initial-support stops, one budget stop and 0/10 freshly associated convergence
remain. New physical trajectory accuracy and vehicle integration remain
unverified. Subjective maturity stays **about 20%** versus mature driving OSS;
**30% and 50% remain unmet**.

The [extended fixed-source correction budget](extended-slope-refinement.md)
retains every original observation, all 97 prior corrections and complete cap 30
reports. Nine extra corrections converge on desk2 source 160 under unchanged full-proposal
criteria, bringing own convergence to 6/10. Fresh convergence stays 0/10 and the
changed window's fresh residual worsens; four initial selected-support guards
remain. Larger bounded computation adds no measured information, physical
trajectory accuracy or vehicle integration. Subjective maturity stays
**about 20%** versus mature driving OSS; **30% and 50% remain unmet**.

The [fitted-field admission comparison](slope-admission-refinement.md) restores
three desk2 windows while retaining every old source and its original metric
weight. Local convergence reaches 9/10 on the same viewed windows, with one
room center-support stop and 0/10 fresh convergence remaining. The larger
population contains residuals from existing frames, changing objective/prior
balance without new independent measurements. Physical trajectory accuracy
and vehicle integration remain unverified. Subjective maturity stays
**about 20%** versus mature driving OSS; **30% and 50% remain unmet**.

The [read-only terminal objective diagnosis](depth-objective-bridge.md)
reconstructs the saved fixed/fresh gradient difference in five ordered terms.
Fresh-only sources have median signed scaled projection 0.974 across the nine
fixed-converged windows, with cancellation allowing values outside zero to one;
this is not a causal or accuracy percentage. Even same-model reselection,
including ordinary center normalizers, gives 0/10 converged proposals. The
30 independently checked endpoint roles include 20 archived systems. No pose
update, new physical score or vehicle integration follows this diagnosis.
Subjective maturity stays **about 20%** versus mature driving OSS;
**30% and 50% remain unmet**.

The [three-round continuous reselection experiment](reselected-slope-refinement.md)
retains its failed first comparison: fresh convergence stays 0/10, terminal
fixed convergence falls from 9/10 to 8/10 and both fresh-residual medians worsen.
All first-round reports reproduce the original control, while later selection
retains only 79.48% of initial source identities. Two exact-grid identity
admission disagreements remain explicitly unresolved. No physical score,
native adoption or vehicle capability is added. Subjective maturity stays
**about 20%** versus mature driving OSS; **30% and 50% remain unmet**.

The [conservative cell-boundary guard](boundary-slope-refinement.md) resolves
the two authored identity admission disagreements in a separately frozen variant.
It retains raw C1 interpolation coordinates and values, with explicit losses
in those fixtures. All ten recorded fit/checkpoint reports exactly reproduce
the original 9/10 control, retaining all 60,288 initial source occurrences and
124 corrections. Both fresh convergence checks remain 0/10. The engineering
band is not a universal numerical-stability proof and supplies no new physical
accuracy or driving capability. Subjective maturity stays **about 20%** versus
mature driving OSS; **30% and 50% remain unmet**.

The [legacy/fitted center union](fitted-center-refinement.md) reaches local
fixed-objective convergence on all ten viewed windows, retaining every original
source and metric value and admitting 20,140 additional occurrences from the
same frames. Both fresh convergence checks remain 0/10, and desk2's median fresh
RMS worsens. These fits therefore fail the original progression rule and add
no measured physical trajectory accuracy or vehicle integration. Subjective
maturity stays **about 20%** versus mature driving OSS;
**30% and 50% remain unmet**.

The [fitted-center reselection comparison](reselected-center-refinement.md)
lowers both median fresh residuals but leaves both fresh convergence checks at
0/10 and reduces terminal fixed convergence from 10/10 to 7/10. All first-round
fits reproduce their controls; later rounds expose a selected-support line-search
stop and two inner-budget stops. Source retention across reselection is 79.55%,
without new independent observations or physical accuracy evidence. The failed
progression adds no vehicle capability. Subjective maturity stays **about 20%**
versus mature driving OSS; **30% and 50% remain unmet**.

The [two-pixel admission comparison](interior-center-refinement.md) identifies
one archived source crossing missing support after 1.07 pixels of image motion.
It removes unsupported candidates and improves terminal fixed convergence from
7/10 to 8/10 while retaining 99.08% of initial sources and their exact metrics.
Both original-domain checks and new-policy fresh convergence remain 0/10; both
fresh-residual medians worsen relative to the preceding outer comparison. This
repairs an observed support failure without physical accuracy or vehicle
integration evidence. Subjective maturity stays **about 20%** versus mature
driving OSS; **30% and 50% remain unmet**.

The [maintained native-depth comparison](native-depth-regression.md) returns to
complete Rust trajectories and a reusable default-adoption gate. Four implemented
candidate changes fail that gate; no accuracy improvement is adopted. Exact
default replay retains64/179 room and161/179 desk2 accurate updates. Configurable
native code and stronger regression checks do not resolve physical accuracy or
driving integration. Subjective maturity stays **about 20%** versus mature OSS.

The subsequent [native normal-estimation experiment](native-depth-regression.md#normal-noise-diagnosis-and-rejected-native-change)
reduces local planar-normal variation but regresses complete accuracy to63/179
room and85/179 desk2. Its strict-support first attempt also fails existing motion
controls; the maintained correction restores those controls without improving
physical accuracy. Neither attempt is adopted. The default remains64/179 and
161/179, and subjective maturity remains **about 20%** versus mature driving OSS.

The [native XYZ model](projective-vertices.md) preserves measured points through
fusion, view changes and pyramid reduction, preventing a known-plane resampling
deformation. It scores 67/179 room and 85/179 desk2, so it fails continuous adoption
and leaves the default unchanged. Transport checks also catch invalid
configuration acceptance previously masked by an EOF error. These corrections
add no qualified vehicle capability or sustained accuracy gain; subjective
maturity remains **about 20%** versus mature driving OSS.

The [fused RGB-D mode](projective-rgbd-fusion.md) recovers a synthetic textured
plane's otherwise unobservable motion and handles independent image/depth
freshness with explicit reference ownership. Its viewed room result improves
to 80/179, but desk2 regresses to 75/179. All updates are available after the
asynchronous-image correction; the original complete accuracy and cross-record
adoption gates remain unmet. The mode is not the default and supplies no
qualified vehicle capability. Subjective maturity stays **about 20%**.

The maintained [RGB-D exposure correction](projective-rgbd-exposure.md) separates
brightness gain/offset from pose and improves the viewed RGB-D results to
84/179 room and 124/179 desk2. Its synthetic invariance and rejection controls
pass, but desk2 remains worse than the 161/179 depth-only default and adoption
fails. This repairs a specific measurement-model limitation without qualified
sustained localization or driving integration. Subjective maturity remains
**about 20%** versus mature driving OSS.

The [fixed-support step-control policy](projective-step-control.md) adds measured
nonlinear update checks with independent reconstruction and image-boundary
controls. Despite passing those checks, it regresses continuous accuracy to
83/179 room and 67/179 desk2. It remains explicitly opt-in; both previous
defaults are preserved. Local descent is not qualified localization accuracy,
and subjective maturity remains **about 20%** versus mature driving OSS.

The [causal motion initializer](projective-motion.md) handles actual acquisition
intervals and excludes rejected observations from pose history. Its corrected
SE(3) implementation removes a numerical frame-conversion error and passes
independent history/fault controls. Depth accuracy is67/159; RGB-D is75/169,
with room regression and unmet maximum-error gates. Both defaults remain
unchanged. Timing diagnostics identify model-predicted image displacement, not
validated synchronization error or compensation. Subjective maturity stays
**about 20%** versus mature driving OSS.


The [causal RGB/depth timing compensation](projective-timing.md) now resamples
both images at their own acquisition times from previous accepted poses, with
independently checked Jacobians and reference-rate ownership. Own-time plane
accuracy improves from 2/8 to 8/8, while complete viewed results reach 80/179 room
and 171/179 desk2. Maximum rotation error still fails adoption; ordinary RGB-D
also retains a better room count. The default stays unchanged, with no held-out
or automotive qualification. Subjective maturity remains **about 20%** versus
mature driving OSS.


The mandatory [directional information and report-integrity repairs](projective-information.md)
reject six previously accepted weak planar fits, preserve all 1,080 historical
sensor outcomes, and prevent partial JSON reports from being published. The
first new official plant candidate fails metadata qualification; a separately
frozen RPY recording passes raw reconstruction but scores 74/179 depth and 61/179
compensated RGB-D. Nominal conditional uncertainty has 0% observed coverage;
empirical variance inflation reaches 100% with 1.55–2.71 m positional radii and
fails its usefulness gate. Neither is qualified uncertainty. These default
robustness improvements and retained new failures do not close the 30% waypoint;
subjective maturity remains **about 20%** versus mature driving OSS.

The later [surface-residual and bidirectional-matching comparison](projective-surface.md)
retains those original failures. The new bidirectional estimator reduces RPY's
maximum rotation error to about 0.156 rad, but scores 64/159/75 accurate updates
in room/desk2/RPY and fails the unchanged adoption floors. Measured-normal
averaging alone regresses desk2 further. Both stay opt-in and supply no calibrated
uncertainty. Implementation and independent checks do not justify a percentage
increase: maturity remains **about 20%**, with the 30% waypoint unmet.

The [relative axial-depth weighting study](projective-range.md) finds increasing
depth-residual dispersion in three viewed recordings. Its two native modes improve
room/RPY counts but substantially regress desk2; both fail the fixed adoption rule
and retain the original default. Standardizing the robust loss does not resolve
that failure. Their conditional synthetic checks and raw reconstruction establish
implementation integrity, without calibrated physical noise, held-out accuracy or
a maturity increase. The estimate remains **about 20%**, with 30% unmet.

The [visible-surface fusion study](projective-fusion.md) then reduces a constructed
nearest-Z selection bias and implements the corresponding native raster policy.
Reference-transform map probes do not isolate a unique physical drift cause.
Complete viewed scores67/161/74 retain larger maximum position errors than the
default, so adoption fails. Independent raw reconstruction and accepted-state
controls establish implementation integrity, without improving the maturity
estimate: **about 20%**, with the **30% waypoint unmet**.

The subsequent [measured free-space update](projective-visibility.md) correctly
removes a controlled disappearing foreground while retaining support through
missing or occluded observations. Fixed-support map probes separate stored-point
and normal changes algebraically. Recorded scores67/161/74 still increase maximum
position error in room/desk2, so the mode remains disabled by default. These
mapping and diagnostic additions do not establish qualified localization;
maturity remains **about 20%**, with the **30% waypoint unmet**.

The [raw camera correction study](projective-camera-fit.md) adds independent
focal/depth derivatives, complete fold audits and a repair for noisy scale
ambiguity. Two corrections fail the frozen stability/validation gate; neither
exports a camera profile or changes the tracker. This improves the calibration
diagnostic while leaving physical calibration and sustained accuracy unresolved.
The engineering estimate remains **about 20%**, with the **30% waypoint unmet**.
