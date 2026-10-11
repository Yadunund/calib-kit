# Working in calib-kit

calib-kit is defined by its docs. Code implements them; tests check them. This holds for people
and agents alike.

| Doc | Says | Changes when |
|---|---|---|
| `docs/intent.md` | why calib-kit exists, for whom, its principles and non-goals | the toolkit's purpose or boundaries change |
| `docs/spec.md` | the rules every calibration follows (`KIT-*`) | a rule for all calibrations changes |
| `docs/<calibration>/intent.md` | why that calibration exists, its outcomes and non-goals | its purpose or boundaries change |
| `docs/<calibration>/spec.md` | its requirements (`ARM-*` for `arm_calibration`) | any of its behaviour changes |
| `docs/templates/` | the shape of every intent and spec | the templates themselves improve |

## Before changing anything

Read `docs/intent.md`, `docs/spec.md`, and the intent and spec of the calibration you are touching.
A change that conflicts with an intent's principles or non-goals needs the intent changed first,
and the person's agreement.

## Changing behaviour

1. **Spec first.** Add or change the requirements the change needs, in the calibration's
   `spec.md` (and its `intent.md` if an outcome, principle or non-goal moves).
2. **Then the code**, doing exactly what the requirements say.
3. **Then the tests.** Each requirement is checked by a test that names its ID in its docstring,
   against simulated measurements whose truth is known. A requirement no test names is not done; a
   behaviour no requirement names should not exist.
4. **Then the README** of the package, which tells a user how to use what the spec defines.

All four land in the same commit.

## A new calibration

Copy `docs/templates/intent.md` and `docs/templates/spec.md` to `docs/<package>/`, and write the
intent, then the spec (with its own requirement prefix), before the package. Add it to the table
in the root `README.md`.

## Writing the docs

- **State what holds now.** No status, no progress, no results of runs or tests, no history:
  git holds history.
- **Requirement IDs are stable.** Never renumber or reuse one; a requirement dropped is deleted.
- **One checkable statement per requirement**, with the numbers the code uses.
- **Standalone.** Nothing here mentions or depends on an application that uses calib-kit.

## Checks and commits

- `pixi run lint`, `pixi run build` and `pixi run test` pass before every commit; CI runs the same.
- Commit to `main`. Messages are short: a subject, and a few lines on why if it is not obvious.
