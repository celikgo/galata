# Galata end-product plan

**Status: proposed scope and action plan, 7 September 2026. These documents do not describe delivered features or qualification evidence.** They extend the [current roadmap](ROADMAP.md) and incorporate the [reliability audit baseline](product/AUDIT_BASELINE.md). Existing capabilities remain defined by the README, registry and verification report.

The proposed end product is an **installable, offline flight-dynamics and control-engineering desktop workbench with Simulink-style visual modeling**, backed by the same C++ numerical engine used by the CLI and automation. A flight-control engineer can import an aircraft model, check its validity, build a controller diagram, run analysis and simulation, compare designs over an operating envelope, and deliver a reproducible review package.

The planning assumption is conventional fixed-wing aircraft first. Product delivery prioritizes macOS, followed by Linux, per user direction; existing Windows engine portability checks remain. Minimum OS/hardware and workload requirements still need discovery. No customer aircraft, procurement authority, team capacity or delivery deadline has been supplied. The scope becomes a release commitment only after discovery establishes those constraints. SITL/HIL and generated controller code have explicit expansion tracks; onboard operation and aircraft approval require their own acceptance programs.

| Plan | Purpose |
|---|---|
| [Feature set](product/FEATURES.md) | Prioritized end-user capabilities, baseline gaps, dependencies and acceptance criteria |
| [Simulink-style capabilities](product/SIMULINK_FEATURES.md) | Visual modeling, execution semantics, libraries, debugging, interoperability and code generation |
| [Discovery agenda](product/DISCOVERY.md) | Questions, bounded investigations and decisions needed before committing implementation |
| [Delivery action plan](product/DELIVERY.md) | Milestones, first work packages, owners, dependencies and release gates |
| [Assurance plan](product/ASSURANCE.md) | Numerical/model evidence, risk, traceability, security, release integrity and sustainment |
| [Audit baseline](product/AUDIT_BASELINE.md) | Known defects the first milestone must close |

**The complete customer workflow.** The product must support all of this through the installed application, with repeatable headless execution of the same saved project:

1. Create a project with a defined engineering question, configuration and acceptance criteria.
2. Import model and measured/reference data; inspect units, coordinates, source rights, parameter uncertainty and the supported domain.
3. Find feasible trim points and construct an operating-point grid; inspect solver residuals, unavailable thrust/control authority and invalid points.
4. Build a visual plant/controller/sensor/actuator model using reusable blocks and subsystems; reject invalid wiring, unsupported feedback equations and inconsistent rates before running.
5. Linearize, inspect modes and frequency response, design continuous or sampled controllers, and preserve numerical diagnostics and assumptions.
6. Simulate reference maneuvers, disturbances, saturation, delays and declared faults; inspect scopes and demonstrate numerical convergence.
7. Run parameter/envelope/uncertainty campaigns; compare candidates against requirements and review every failure or unresolved case.
8. Export a review package containing model revisions, inputs, results, evidence, limitations and runtime identity; reopen and verify it on another supported installation.

**V1 acceptance is an outcome, not a feature count.** A release candidate qualifies as the proposed finished workbench when an engineer outside the development team can install it without a compiler, complete the workflow above on the agreed representative aircraft/data, reproduce it through the CLI, understand failed and unresolved results, and hand a reviewer an independently checkable package. Independent benchmark and customer acceptance evidence must cover the advertised numerical and model domain. No unresolved defect may allow an invalid/unresolved result to masquerade as a passed engineering requirement.

| Product dimension | Required end-product outcome |
|---|---|
| Trustworthy analysis | Estimated, numerically bounded, unresolved, outside-domain and failed results are visibly distinct; required diagnostics survive every adapter |
| Model-based authoring | Functional block editor, hierarchy, parameters, sample-time checking, reusable model libraries and documented supported semantics |
| Domain capability | Validated model packages and continuous/sampled-control studies across an explicitly declared fixed-wing domain |
| Simulation and comparison | Repeatable scenarios, convergence evidence, uncertainty campaigns, linked plots and failure investigation |
| Usability | Guided project flow, undo/redo, useful diagnostics, responsive views, recoverable runs, keyboard/accessibility testing and task-based documentation |
| Interoperability | Stable project/data contracts, CLI/C++ automation, selected Python/data adapters; explicit capability/loss reports for imported models |
| Delivery | Supported installers, offline operation, verified runtime inventory, signed distribution, upgrades/rollback and schema migration |
| Support | Named ownership, support policy, maintained reference cases, issue reproduction bundles, security response and supported-release maintenance |

**Architecture direction.** Preserve the current numerical modules and study capability registry. Introduce a versioned engineering artifact contract that carries data, dimensions, units, coordinates, numerical evidence, model validity, provenance and requirement evaluation. Both the GUI and CLI consume that contract. Graphical blocks lower into a simulation intermediate representation (IR); no numerical algorithm lives only in the UI. The study DAG orchestrates trim, synthesis, simulation and reporting; the compiled block model defines the dynamics inside a simulation job. Cyclic signal graphs cannot simply be passed to the existing acyclic pipeline scheduler.

A numerical worker executes validated models separately from the desktop presentation process, with explicit cancellation, resource limits and result publication. Process separation improves fault containment; it does not by itself sandbox native extensions. Project/model revisions and run packages are authoritative. Search indexes and optional caches are rebuildable. Execution identity must include the numerical runtime and dependencies, not merely the GUI launcher. The desktop framework, storage format, numeric backend changes and interoperability routes require measured discovery decisions before adoption.

**Product boundaries.** V1 concentrates on the engineering workflow above. Rotorcraft/VTOL, spacecraft, CFD, aeroelasticity, high-fidelity stall/spin, general multiphysics/DAE modeling, unrestricted executable blocks, broad Simulink compatibility and arbitrary hardware targets are separately costed expansions. State-machine logic, restricted code generation and SITL/HIL are included in the feature plan with prerequisites. Their deferral from the first acceptance boundary is a sequencing decision, not deletion from the intended product direction. Update the relevant ADR before changing existing architectural commitments, including plugin ABI and deterministic solver policies.

The reference ecosystem confirms the breadth of the requested direction: Simulink combines graphical modeling with simulation, while Aerospace Blockset adds aircraft-oriented models and analysis. Galata's proposed differentiator is a focused offline flight-control workflow with inspectable numerical/model evidence and reproducible review packages. This positioning is a hypothesis to validate with engineers, not a measured market claim. [Simulink overview](https://www.mathworks.com/products/simulink.html), [Aerospace Blockset documentation](https://www.mathworks.com/help/aeroblks/index.html).

**Immediate decision.** Begin discovery and audit closure in parallel. Freeze the v1 customer workflow and acceptance contract at M1. Prove one graphical aircraft-control study end to end at M2 before expanding the block library or model domain. The [delivery plan](product/DELIVERY.md) defines the work and evidence required for each subsequent milestone; a calendar commitment follows staffing and the discovery results.

Implementation is tracked in [M0 implementation status](product/IMPLEMENTATION.md).
The [proposed executable-model architecture](architecture/EXECUTABLE_MODEL.md)
specifies the separate compiler/runtime needed by the Simulink-style workbench.

The authorized next increment is tracked in [M1 implementation](product/M1_IMPLEMENTATION.md),
with [current contracts](product/M1_CONTRACTS.md): macOS first, Linux next.
