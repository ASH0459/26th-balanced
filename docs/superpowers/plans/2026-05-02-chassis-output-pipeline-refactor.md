# Chassis Output Pipeline Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the chassis control loop so each phase decides control intent and mode, while one final stage produces `wheel_T`, `Tbl_t`, `Fbl_t`, and `joint_T`.

**Architecture:** Keep the refactor local to `Users/Application/APP_Chassis/App_Chassis_Task.cpp` unless a type must be shared. Split the current pipeline into explicit layers: base output computation, phase policy selection, support-force computation, and final output emission. Preserve a small number of direct-joint special cases (`FLIP`, `INIT_FOLD`, `STEP_UP_SWING/HOLD`) instead of forcing every phase into one generic formula.

**Tech Stack:** Embedded C/C++, STM32 firmware, FreeRTOS task loop, CMake presets (`Debug`), existing WBR/LQR control code.

---

## Context

Current output ownership is hard to follow because one control quantity may be assigned multiple times in one loop:

- `chassis_lqr_power_control()` writes `wheel_T` and `Tbl_t`.
- `chassis_init_standup()` may overwrite wheel, leg, or direct joint output.
- `chassis_calc_support_force()` computes `Fbl_t`, but also zeros `wheel_T` and `Tbl_t` in retract-like phases.
- `chassis_apply_joint_output()` contains another phase switch and may rewrite `wheel_T` / `Tbl_t`.

This is behaviorally valid, but the responsibility boundaries are blurred:

- Function names do not match what they actually mutate.
- A reader cannot quickly answer “who owns the final `wheel_T` this cycle?”
- Special phases are mixed into normal force-calculation code.

## Non-Goals

- Do not retune gains, thresholds, or timing constants.
- Do not change state machine semantics for `INIT`, `STEP_UP`, `JUMP`, or `FLIP`.
- Do not move this logic into new files unless the `.cpp` becomes unmanageable during the refactor.

## Success Criteria

- One clearly documented stage finalizes `wheel_T`, `Tbl_t`, and `Fbl_t` for the current control loop.
- `chassis_calc_support_force()` no longer mutates `wheel_T` or `Tbl_t`.
- Phase logic decides mode and override policy instead of scattering direct output writes across unrelated functions.
- Direct `joint_T` writes remain only for genuinely special modes and are explicitly labeled as such.
- `cmake --build --preset Debug` succeeds after the refactor.

## File Structure

**Primary file:**
- Modify: `Users/Application/APP_Chassis/App_Chassis_Task.cpp`

**Optional only if needed for clarity:**
- Modify: `Users/Application/APP_Chassis/App_Chassis_Task.h`
- Modify: `Users/Application/APP_Chassis/App_Chassis_Task_Helpers.cpp`
- Modify: `Users/Application/APP_Chassis/App_Chassis_Task_Helpers.h`

## Proposed Target Structure

Keep the control loop shape readable:

1. `chassis_lqr_power_control(...)`
   - Compute only base `wheel_T` and base `Tbl_t`.
   - Do not own final phase-specific zeroing.

2. `chassis_init_standup(...)`
   - Only update INIT-phase state and, when necessary, fill direct `joint_T`.
   - Do not silently own the final normal-mode `wheel_T` / `Tbl_t` policy.

3. `chassis_calc_support_force(...)`
   - Compute only `Fbl_t` and related feedforward terms.
   - No wheel or `Tbl_t` writes.

4. `chassis_finalize_output_policy(...)` or equivalent local logic
   - Decide the current output mode from state and phase.
   - Examples:
     - `NORMAL_WBR`
     - `RETRACT_LEG_ONLY`
     - `DIRECT_JOINT_ALREADY_WRITTEN`
     - `STEP_STAND_NO_WHEEL`

5. `chassis_apply_joint_output(...)`
   - Consume the selected mode and the already-computed `wheel_T` / `Tbl_t` / `Fbl_t`.
   - Be the final place where “what leaves this loop” is decided.

## Mode Ownership Rules

These rules should hold after the refactor:

- `wheel_T`
  - Produced by base LQR.
  - Zeroed only in the final output-mode stage for retract-like or direct-joint phases.

- `Tbl_t`
  - Produced by base LQR.
  - Zeroed or replaced only in the final output-mode stage.

- `Fbl_t`
  - Produced by support-force logic.
  - Never mixed with wheel/Tbl zeroing logic inside support-force code.

- `joint_T`
  - Written directly only for special phases that are already not standard WBR output:
    - `FLIP`
    - `INIT_FOLD`
    - `STEP_UP_SWING`
    - `STEP_UP_HOLD`

## Task 1: Introduce Explicit Output Ownership

**Files:**
- Modify: `Users/Application/APP_Chassis/App_Chassis_Task.cpp`

- [ ] **Step 1: Add a local output-mode enum near the static function declarations**

```cpp
typedef enum
{
    CHASSIS_OUTPUT_MODE_NORMAL_WBR = 0,
    CHASSIS_OUTPUT_MODE_RETRACT_LEG_ONLY,
    CHASSIS_OUTPUT_MODE_STEP_STAND,
    CHASSIS_OUTPUT_MODE_DIRECT_JOINT
} chassis_output_mode_e;
```

Add a short comment above the enum:

```cpp
// Add a new output mode only when the final output mapping strategy changes.
// If only scales/biases/gates differ, keep the same mode and adjust policy locally.
```

- [ ] **Step 2: Add a small selector function declaration and definition in `App_Chassis_Task.cpp`**

```cpp
static chassis_output_mode_e chassis_select_output_mode(const Chassis_Move *chassis_move_control_loop);
```

Implementation must encode the current intent already present in comments and branches:
- `INIT_RETRACT` and `STEP_UP_RETRACT` -> `CHASSIS_OUTPUT_MODE_RETRACT_LEG_ONLY`
- `STEP_UP_STAND` -> `CHASSIS_OUTPUT_MODE_STEP_STAND`
- `INIT_FOLD` -> `CHASSIS_OUTPUT_MODE_DIRECT_JOINT`
- everything else -> `CHASSIS_OUTPUT_MODE_NORMAL_WBR`

Add an implementation comment that these phases do not reach the selector because they return earlier in `chassis_control_loop()`:
- `CHASSIS_FLIP`
- `STEP_UP_SWING`
- `STEP_UP_HOLD`

- [ ] **Step 3: Build to verify only the enum/function addition compiles cleanly**

Run:

```bash
cmake --build --preset Debug
```

Expected:
- Build completes successfully

Warning:
- Task 2 and Task 3 form one behavior-preserving relocation pair.
- After Task 2 alone, retract-like and `INIT_FOLD` phases may temporarily lose the relocated `wheel_T` / `Tbl_t` zeroing until Task 3 is completed.
- Do not stop for on-hardware validation between these two tasks.

## Task 2: Make Support-Force Code Pure

**Files:**
- Modify: `Users/Application/APP_Chassis/App_Chassis_Task.cpp`

- [ ] **Step 1: Remove `wheel_T` / `Tbl_t` writes from `chassis_calc_support_force()`**

The branch around current retract handling should stop doing this:

```cpp
chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
```

The function should still compute the correct `Fbl_t` for:
- retract-like phases
- jump takeoff
- jump ready-land
- normal support-force mode

- [ ] **Step 2: Remove `INIT_FOLD` from the retract-force support branch**

Today `init_retract_force` mixes `INIT_FOLD` and `INIT_RETRACT`. After the refactor, `INIT_FOLD` should no longer be treated as a retract-leg-only force mode.

The condition should conceptually become:

```cpp
const bool_t init_retract_force =
    (chassis_move_control_loop->state == CHASSIS_INIT &&
     chassis_move_control_loop->init_phase == CHASSIS_INIT_RETRACT) ||
    (chassis_is_step_up_active(chassis_move_control_loop) &&
     chassis_move_control_loop->step_up_phase == STEP_UP_RETRACT);
```

This keeps `INIT_FOLD` out of the retract-only `Fbl_t` path and aligns it with `DIRECT_JOINT` ownership.

- [ ] **Step 3: Remove `wheel_T` / `Tbl_t` clearing from `chassis_init_standup()`**

The `INIT_FOLD` branch currently zeros wheel/Tbl directly before writing direct `joint_T`. Those writes must move to the final output stage so `chassis_init_standup()` only:
- updates INIT phase state
- computes direct `joint_T` when needed

- [ ] **Step 4: Update the function comment to match the new responsibility**

Replace vague wording with something explicit, for example:

```cpp
/**
 * @brief Compute left/right leg support force requests (Fbl_t only)
 */
```

- [ ] **Step 5: Build to verify this responsibility split compiles**

Run:

```bash
cmake --build --preset Debug
```

Expected:
- Build completes successfully

## Task 3: Centralize Final Wheel/Tbl Policy

**Files:**
- Modify: `Users/Application/APP_Chassis/App_Chassis_Task.cpp`

- [ ] **Step 1: Call `chassis_select_output_mode()` inside `chassis_apply_joint_output()`**

Add a local mode variable at the top:

```cpp
const chassis_output_mode_e output_mode =
    chassis_select_output_mode(chassis_move_control_loop);
```

- [ ] **Step 2: Convert the final output dispatcher to `switch (output_mode)`**

Do not keep the old phase-driven `if` / `else if` chain once `output_mode` exists.

The final-output stage should have one mutually-exclusive dispatcher:

```cpp
switch (output_mode)
{
case CHASSIS_OUTPUT_MODE_RETRACT_LEG_ONLY:
    ...
    break;
case CHASSIS_OUTPUT_MODE_STEP_STAND:
    ...
    break;
case CHASSIS_OUTPUT_MODE_DIRECT_JOINT:
    ...
    break;
case CHASSIS_OUTPUT_MODE_NORMAL_WBR:
default:
    // default: NORMAL_WBR is the safety fallback; when adding a new
    // enum value, add an explicit case here as well.
    ...
    break;
}
```

The selector remains the single source of truth; the dispatcher must not re-derive phase meaning from scratch.

- [ ] **Step 3: Move retract-like wheel/Tbl zeroing into the final output stage**

The `RETRACT_LEG_ONLY` branch should explicitly produce:

```cpp
chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;

APPLY_WBR_JOINT_OUTPUT(chassis_move_control_loop,
                       chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t,
                       0.0f,
                       chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t,
                       0.0f);
```

- [ ] **Step 4: Place INIT wheel gating in a final-stage preprocessing block**

Do not hide INIT wheel gating inside the `NORMAL_WBR` branch, because that creates an implicit sub-mode inside `NORMAL_WBR`.

Move the existing `chassis_apply_init_wheel_theta_gate()` call out of
`chassis_lqr_power_control()` and place it in the final output stage instead.
Do not leave the old call in place, or wheel output will be attenuated twice.

Use a small preprocessing block immediately before the `switch (output_mode)` dispatcher, for example:

```cpp
// Final-output preprocessing: INIT stand-up may attenuate wheel output
// before the mapping strategy is applied.
if (chassis_move_control_loop->state == CHASSIS_INIT)
{
    chassis_apply_init_wheel_theta_gate(chassis_move_control_loop);
}
```

Add a short comment near `chassis_apply_init_wheel_theta_gate()` noting the timing assumption:

```cpp
// Assumes wheel_T has not been modified since base LQR output generation.
```

- [ ] **Step 5: Move `STEP_UP_STAND` wheel suppression into the same final output stage**

This branch should continue to zero both wheels, then compute the angle-only `Tbl_t` correction, then map to joints.

- [ ] **Step 6: Keep direct-joint phases explicit and isolated**

`DIRECT_JOINT` mode should do no extra WBR remapping and should preserve the `joint_T` written earlier in the loop.

It must also explicitly clear stale wheel/Tbl values:

```cpp
chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
```

- [ ] **Step 7: Build to verify the centralized ownership compiles**

Run:

```bash
cmake --build --preset Debug
```

Expected:
- Build completes successfully

## Task 4: Tighten Control-Loop Narrative

**Files:**
- Modify: `Users/Application/APP_Chassis/App_Chassis_Task.cpp`

- [ ] **Step 1: Update the `chassis_control_loop()` comments so the stages match reality**

The top-of-function comment should describe:
- base output generation
- phase-specific state updates
- support-force computation
- final-output preprocessing + mode application
- output limiting and feedback save

- [ ] **Step 2: Add short ownership comments for the remaining intentional write sites**

Document these intentional exceptions so future reviewers do not treat them as ownership leaks:

- `chassis_apply_leg_tbl_angle_attenuation()` is a legal final-stage `Tbl_t` adjustment.
- The retract-like `Fbl_t = ... + 100` branch uses an intentional upward force bias during leg retraction.

Example comments:

```cpp
// Final output stage: this is the only place that decides the
// cycle's final wheel_T / Tbl_t / joint_T ownership.

// Legal final-stage Tbl_t postprocess: attenuate leg lateral torque
// near large leg angles without changing ownership stage.

// Retract-stage support-force bias: extra upward force to avoid
// insufficient support while legs are being pulled in.
```

- [ ] **Step 3: Run a final build and sanity scan**

Run:

```bash
cmake --build --preset Debug
rg -n "wheel_T = 0.0f|Tbl_t = 0.0f" Users/Application/APP_Chassis/App_Chassis_Task.cpp
```

PowerShell fallback if `rg` is unavailable:

```powershell
Get-ChildItem Users\Application\APP_Chassis\App_Chassis_Task.cpp |
    Select-String -Pattern "wheel_T = 0.0f|Tbl_t = 0.0f"
```

Expected:
- Build completes successfully
- Remaining zeroing sites are obviously tied to:
  - STOP
  - FLIP
  - direct-joint special phases
  - final output-mode application
- Remaining `Tbl_t` post-processing is obviously tied to:
  - final output-stage attenuation (`chassis_apply_leg_tbl_angle_attenuation`)

## Review Checklist for the Other AI

- Does the plan preserve current behavior while improving ownership clarity?
- Is `chassis_calc_support_force()` truly the right place to stop mutating `wheel_T` / `Tbl_t`?
- Should `STEP_UP_STAND` stay as a special final-output mode, or be folded into normal WBR output with wheel gating?
- Are there any remaining phases that still secretly own final outputs outside the designated final-output stage?
- Is the chosen enum small enough, or is it hiding too many distinct behaviors under one abstraction?
- Does `INIT_FOLD` now cleanly belong to `DIRECT_JOINT` instead of the retract-force path?
- Does `chassis_apply_init_wheel_theta_gate()` now have one obvious owner in the final policy stage?
- Does the final-output stage consume `output_mode` with a `switch`, not a re-derived phase `if` / `else` chain?
- Is INIT wheel gating implemented as preprocessing, not as a hidden `NORMAL_WBR` sub-mode?
- Is `chassis_apply_leg_tbl_angle_attenuation()` clearly documented as a legal final-stage `Tbl_t` adjustment?
- Does the sample or implementation comment explain that `default:` is a NORMAL_WBR safety fallback?

## Decided Constraints

- `CHASSIS_JUMP` stays on `CHASSIS_OUTPUT_MODE_NORMAL_WBR`.
  Reason: jump changes support-force policy, but not the final WBR output mapping strategy.

## Open Questions

1. Should `chassis_init_standup()` eventually stop writing direct `joint_T`, or is `INIT_FOLD` special enough to keep direct control permanently?

## Verification Summary

Minimum verification for this refactor:

- Build after each task:

```bash
cmake --build --preset Debug
```

- Manual source audit:

```bash
rg -n "wheel_T =|Tbl_t =|Fbl_t =" Users/Application/APP_Chassis/App_Chassis_Task.cpp
```

PowerShell fallback:

```powershell
Get-Content Users\Application\APP_Chassis\App_Chassis_Task.cpp |
    Select-String -Pattern "wheel_T =|Tbl_t =|Fbl_t ="
```

- Expected final ownership:
  - `wheel_T` base generation in `chassis_lqr_power_control()`, final suppression only in final output stage or true direct-joint phases
  - `Tbl_t` base generation in `chassis_lqr_power_control()`, final suppression/replacement only in final output stage or true direct-joint phases
  - `Fbl_t` generation in `chassis_calc_support_force()` only
  - `chassis_apply_leg_tbl_angle_attenuation()` remains an explicit final-stage `Tbl_t` postprocess, not a hidden ownership leak

Plan complete and saved to `docs/superpowers/plans/2026-05-02-chassis-output-pipeline-refactor.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
