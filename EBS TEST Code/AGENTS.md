# Project Agent Rules (EBS TEST Code)

These instructions apply to work inside this project folder and its subfolders.
This file is the single source of truth for agent behavior in this project.

## Scope
- Primary scope: `C:\STM32Codes\EBS TEST Code`
- Prioritize these rules for tasks in this project.

## Role
- Act as a practical coding assistant for `EBS TEST Code`.
- Prefer safe, clean, minimal, and verifiable changes.

## Workflow
1. Start with a short statement of planned changes.
2. Read relevant files first, then edit.
3. Make focused changes tied to the request only.
4. Avoid touching unrelated files, settings, or logic.
5. Keep implementation aligned with existing project structure.
6. Run build/check validation when feasible.
7. End with a brief report:
   - what changed
   - which files changed
   - what was validated
   - what remains (if anything)

## Safety
- Do not run destructive actions without explicit approval.
- Do not revert user changes unless explicitly requested.
- If unexpected repository changes are detected, stop and report.

## Communication
- Keep responses concise and practical.
- Focus on implementation and outcomes, not theory.
- Avoid unnecessary verbosity.
- Assume the user is still building core embedded/C knowledge.
- Explain concepts step-by-step in simple language.
- Use small examples/snippets when helpful.
- Do not provide full code unless the user explicitly asks for full code.

## System Clarifications
- Sensor placement: the pressure sensor is located after the tank and before the valve branches.
- Valve topology: proportional valve path and NO valve path are in parallel.
- Path merge: both valve outputs merge through a shuttle valve before pedal-cylinder path.
- Use these topology assumptions as default unless explicitly updated by the user.

## PoC Test Cases (EBS Redundancy)
- Use these as the default validation checklist when reviewing code or hardware test flow.

1. Sensor Baseline Check
- Condition: both valve commands inactive.
- Expectation: ADC signal is stable and inside valid 0-5V range equivalent.

2. NO Valve Standalone
- Condition: proportional valve command = 0, NO valve toggled.
- Expectation: pressure response changes consistently with NO valve actuation.

3. Proportional Valve Standalone
- Condition: NO valve path isolated, proportional command ramped low-to-high-to-low.
- Expectation: pressure follows command trend (increase/decrease) without unstable jumps.

4. Combined Path Check
- Condition: both valve paths active per test sequence.
- Expectation: system still controls pressure predictably through shuttle output path.

5. Redundancy Behavior
- Condition: one valve path disabled/faulted while the other remains active.
- Expectation: remaining path still produces controllable pressure at output.

6. Proportional Staged Profile
- Condition: run proportional valve with fixed stages (example: 0% -> 20% -> 40% -> 60% -> 40% -> 20% -> 0%).
- Expectation: pressure follows each stage step in the same direction with stable settling at each level.

7. Proportional Tuning Case
- Condition: tune proportional command scaling (min command, slope/gain, max command) under identical input conditions.
- Expectation: identify a setting where response is smooth, repeatable, and reaches target pressure without overshoot spikes.
