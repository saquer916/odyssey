# Odyssey — FTC Pathing Library: Full Project Handoff (v2)

> **Purpose of this document:** Complete context dump for continuing the Odyssey project in a fresh chat. Project goal, working relationship, architecture, every class, the math and *why*, key decisions, current state, open problems, and the hardware plan. Read it top to bottom before answering anything.
>
> **v2 supersedes v1.** Hardware has arrived, the push test **passed**, and the acceleration/centripetal feedforward path is now largely built. Sections marked **[NEW in v2]** or **[CORRECTED in v2]** changed since the last handoff. Two v1 claims were traced and found **wrong** — see §14.

---

## 0. HOW TO WORK WITH THIS PERSON (read this first)

- They are **head of software on an FTC robotics team**, building this pathing library largely solo. They started as a **Java beginner** but have since built a substantial, mathematically sophisticated library — clearly capable, still learning Java idioms and conventions.
- **They write ALL the implementation code themselves.** Your job: explain concepts clearly, spec out methods (what it does, the algorithm, inputs/outputs, every downstream effect), and **review/debug code they paste**. Do **not** write implementation code for them unless they explicitly ask. Standing exceptions they *did* ask for: **test classes** and the **simulation harness** — those you can write in full. **An OpMode is not a test class.** Do not offer to write OpModes.
- **They respond well to:** plain-language / ELI5 explanations, worked-out arithmetic, diagrams, concrete traced examples, and honest pushback.
- **They are frustrated by (and you must avoid):**
    - Hand-waving and vague answers.
    - "We'll fix that later" / deferring a problem instead of facing it. **Face problems head-on. State them fully when they come up.**
    - Asserting something (a formula, a fix, a diagnosis) **without actually verifying it first** — check the units, do the arithmetic, trace an example, *before* stating a conclusion. Claims made and then walked back erode trust badly.
    - Over-hedging / excessive disclaimers.
    - Repeating the same "here's the next step" framing after they've asked a direct question. Answer the question asked.

### 0.1 **[NEW in v2] THE TRACE-ALL-EFFECTS RULE**

The person asked for this explicitly and forcefully. It is binding:

> **Every answer must trace ALL effects at EVERY step** — what breaks, what silently changes, what interacts with what downstream, what compiles-but-is-wrong. Never defer with "that's the next step." If something has a consequence three steps later, state it when it comes up, not when you arrive there.

This applies to everything, not just one topic. When speccing a method: state its effect on existing behavior, on encapsulation, on tests, on performance, on other subsystems, and on tuning. When reviewing code: state every bug, every silent failure mode, and every downstream interaction, in one pass.

### 0.2 Other standing behavioral notes

- **When something is load-bearing** ("this is the bug," "this formula is right," "centripetal doesn't belong here"), **do the actual work to confirm it before saying it** — numbers, units, a traced case. If you haven't verified it, say so plainly rather than asserting with false confidence.
- **Don't build a method until the code that uses it exists.** This killed several speculative methods (`Point` class, `subtractFrom`, `raiseToThePowerOf`, `getTangentAngleFromVector`, `getAngleFromOther`). When in doubt, wait for a caller.
- **Ask, don't assume.** If a prerequisite may not have been done, ask straight up rather than assuming either way.
- **They want library-quality code** — they intend to **open-source this**. Configurable (tuning values as parameters/constants, not hardcoded), robust (edge cases handled, no crashes on degenerate input), properly encapsulated, and **attribution preserved for any copied code**.

---

## 1. THE PROJECT

**What it is:** A custom autonomous **pathing library** for FTC, named **`odyssey`**, in Java. Generates smooth curved paths from Bézier curves and drives a robot along them accurately.

**Goal in one sentence:** Get a robot to follow a smooth curved path from start to finish — **millimeter-accurate, at a controlled speed, holding the line even when bumped.**

**Origin:** The team tried **Pedro Pathing** and found it "not accurate at all." The likely real cause is a **localization/tuning problem, not the follower**. The person chose to build a custom library anyway, with the ambition that it **rivals Pedro**, has a **GUI**, and is eventually open-sourced.

**The genuine differentiator:** Odyssey does **true arc-length parameterization** with a working **distance↔t inverse**, enabling a **distance-indexed velocity profile**. **Pedro does not do arc-length parameterization** (by their own docs, for simplicity) — which is *why* Pedro's speed control is reactive and they later bolted on "predictive braking." Odyssey computes the speed plan from first principles.

**[NEW in v2] Second differentiator, now real:** Odyssey computes **tangential acceleration analytically from the profile** (`a = ½·d(v²)/ds`, exact on ramps) and **centripetal acceleration from path curvature** (`v²κ`), and feeds both as a proper **`kA` acceleration feedforward**. Pedro launders centripetal through an empirically tuned `centripetalScaling` constant. Odyssey's is dimensionally correct and derived.

**Hardware:** goBILDA **Pinpoint** odometry computer + two odometry pods, mecanum drive. Pinpoint outputs a **field-frame pose**. **Robot is now in hand.** Localization is **verified** (§10).

---

## 2. ARCHITECTURE (multi-project Gradle build)

Root project is `FtcDecode`.

```
FtcDecode/                          ← multi-project root
│   settings.gradle                 (includes all four modules)
│
├── FtcRobotController/             ← stock FTC SDK module (untouched)
│
├── TeamCode/                       ← robot project (Android/FTC)
│   └── src/main/java/org/firstinspires/ftc/teamcode/odyssey/
│       ├── localization/
│       │   ├── GoBildaPinpointDriver.java   ← goBILDA's MIT I²C driver
│       │   └── PinpointLocalizer.java       ✅ [NEW in v2] BUILT, verified by push test
│       ├── drive/                           ← [NEW in v2] package
│       │   └── MecanumDrive.java            🟡 [NEW in v2] written; constructor motor config MISSING
│       └── opmodes/  (or wherever OpModes live)
│           ├── PushTest.java                ✅ [NEW in v2] built and RUN — PASSED
│           ├── testr1.java                  ⚠️ single-motor setVelocity test; leaves a motor in
│           │                                   RUN_USING_ENCODER — see §6.6 mode-persistence trap
│           └── FollowerAuto.java            ⏳ [NEW in v2] NOT BUILT — the missing top of the stack
│       (build.gradle: implementation project(':odyssey-core'))
│
├── odyssey-core/                   ← THE LIBRARY (pure Java, NO Android/FTC deps)
│   │   build.gradle → 'java-library', deps: commons-math3:3.6.1, junit:4.13.2
│   └── src/
│       ├── main/java/org/firstinspires/ftc/teamcode/odyssey/
│       │   ├── geometry/
│       │   │   ├── Vector2d.java        ✅ done
│       │   │   └── Pose2d.java          ✅ done
│       │   ├── path/
│       │   │   ├── BezierCurve.java     ✅ done
│       │   │   ├── Path.java            ✅ done
│       │   │   ├── VelocityProfile.java ✅ [UPDATED in v2] + getTargetTangentialAcceleration
│       │   │   └── PathBuilder.java     ⏳ NOT BUILT (deferred, convenience only)
│       │   ├── localization/
│       │   │   ├── Localizer.java       ✅ interface
│       │   │   └── SimulatedLocalizer.java ❌ [BROKEN in v2] calls renamed DriveSignal getters
│       │   ├── follower/
│       │   │   ├── DriveSignal.java     ✅ [UPDATED in v2] now 5-arg, carries accel, getters renamed
│       │   │   └── Follower.java        🟡 [UPDATED in v2] accel block added; ROTATION BUG open
│       │   ├── control/
│       │   │   └── PIDController.java   ✅ done (adapted from C. Grassin MIT PID)
│       │   └── utils/
│       │       └── MathUtils.java       ⚠️ normalizeAngle, min, abstractEPS — 4-arg `max` UNCONFIRMED
│       └── test/java/.../tests/
│           ├── (geometry, bezier, path, heading, closest-t, velocity-profile tests — were passing)
│           └── FollowerSimTest.java     ❌ [BROKEN in v2] won't compile until SimulatedLocalizer fixed
│
└── odyssey-gui/                    ← JavaFX desktop path editor
    └── src/main/
        ├── java/.../odysseygui/
        │   ├── Main.java              ✅ draws field + draggable Bézier control points
        │   └── FieldCoordinates.java  ✅ mm ↔ pixel conversion (Y-flip)
        └── resources/field.png        ← FTC field image (1028×1028)
```

**Key structural rules:**
- `odyssey-core` is **pure Java** — no `org.firstinspires.ftc.*` or Android imports. This is what makes it a standalone library. `GoBildaPinpointDriver`, `PinpointLocalizer`, and **`MecanumDrive` all stay in TeamCode** — they touch `com.qualcomm.*` hardware types.
- **[NEW in v2] Open question for pre-open-source cleanup:** the mecanum *math* (inverse kinematics + feedforward) is pure and could live in `odyssey-core` as a testable helper, with only the `setPower` calls in TeamCode — mirroring how `Localizer` (interface, core) splits from its impls. Deferred deliberately: one class in TeamCode for now, split later.
- The GUI is a **separate desktop app**. Runs via `./gradlew :odyssey-gui:run` (the green play button fails with "JavaFX runtime components are missing" — must use the Gradle task).
- Library package = `org.firstinspires.ftc.teamcode.odyssey` (FTC boilerplate name kept for now; rename before open-sourcing).
- Java level: `odyssey-core` and `TeamCode` are **Java 8** (Android requirement). `odyssey-gui` can be higher. Do NOT bump `odyssey-core` off Java 8.

---

## 3. THE MATH (with the *why*)

### 3.1 Cubic Bézier curves
Four control points P0–P3. Chosen over quadratic because **cubic gives independent control of start heading (P0→P1) and end heading (P2→P3)**. Cubic can also make S-curves, and is a superset of lines/quadratics.

- **Point:** `B(t) = (1−t)³P0 + 3(1−t)²t·P1 + 3(1−t)t²·P2 + t³·P3`
- **Tangent:** `B'(t) = 3(1−t)²(P1−P0) + 6(1−t)t(P2−P1) + 3t²(P3−P2)`
- **2nd derivative:** `B''(t) = 6(1−t)(P2−2P1+P0) + 6t(P3−2P2+P1)`
- **Coefficient form** (cached as fields `a,b,c,d`): `a = −P0+3P1−3P2+P3`, `b = 3P0−6P1+3P2`, `c = −3P0+3P1`, `d = P0`, so `B(t) = a·t³ + b·t² + c·t + d`
- Intuition: a Bézier is **repeated linear interpolation** (De Casteljau); coefficients are Pascal's triangle (1,3,3,1). `t=0`→P0, `t=1`→P3; middle points only "pull."

### 3.2 Arc length (the crown jewel)
**Problem:** `t` is NOT distance — it sweeps 0→1 unevenly.

- **Arc length = `∫₀¹ |B'(t)| dt`** — integrate speed over t.
- **Critical fact:** for a cubic Bézier this has **no closed form** — `|B'(t)| = √(quartic)`, and `∫√(quartic)` is an **elliptic integral**, provably not elementary. Must be numerical. (Regression, inversion, piecewise splitting, topology, and Jacobi-function inversion were all explored and are dead ends for arbitrary curves. Confirmed by Pomax's primer and geometrictools.com "Moving Along a Curve.")
- **Implementation:** `getLength(b)` uses Apache Commons Math **`IterativeLegendreGaussIntegrator`** for `∫₀ᵇ |B'(t)| dt`.
- **Inverse:** `getTAtDistance(dist)` uses **`BrentSolver`** to find `b` where `getLength(b) = dist`.

### 3.3 Closest point (position → t)
Two versions were built:
1. **Exact (Laguerre):** minimize `D(t)=|B(t)−p|²`; `D'(t)` is degree-5; solve with `LaguerreSolver.solveAllComplex`, filter real roots in [0,1], add endpoints, pick smallest. Elegant but **fragile** (throws on degenerate curves).
2. **Sample-then-refine (CANONICAL):** coarse-sample t in steps of `0.01`, keep closest; refine `±0.01` in steps of `1e-5`, clamped to [0,1]. Robust by construction.

**Rationale:** closest-point feeds a PID correction that's approximate anyway, and localization noise dwarfs the precision difference — robustness > elegance here. (Arc length, by contrast, sets the *target position* directly, so it needs precision. That's why it got the rigorous treatment.)

Uses **squared distance** (`dot(delta,delta)`) to skip the sqrt when only comparing.

### 3.4 Curvature
`κ(t) = |B'(t) × B''(t)| / |B'(t)|³` — the 2D cross product `(ax·by − ay·bx)` gives a scalar. **Note the `Math.abs`** — magnitude only; the *sign* (which side it bends) is discarded. That's fine because direction comes from `getCentripetalVector`, not from the sign (§3.6).

### 3.5 Velocity profile (distance-indexed)
Speed at distance X is limited by three things; take the **minimum**:
1. **Max velocity.**
2. **Curve limit:** `v = √(a_c / κ)` where `a_c` = max centripetal accel. (κ≈0 → no limit → maxVelocity; guard the divide.)
3. **Acceleration/braking:** `v = √(v₀² + 2·a·d)` — distance-based, which is why it fits a distance-indexed profile.

Built in **three separate passes**:
- **Pass 1 (curve limits, local):** `v = min(maxVel, √(a_c/κ))` at each sample.
- **Pass 2 (forward):** `v[0]=0`; walk up, cap by `√(v[i−1]² + 2·maxAccel·ds)`.
- **Pass 3 (backward):** `v[last]=0`; walk down, cap by `√(v[i+1]² + 2·maxBrake·ds)`.

Each pass only lowers values, so they stack. `ds` is read from the distances array (not assumed to be `step`) to handle the clamped last gap.

**Interpolation subtlety (real bug, found & fixed):** `getTargetVelocity` must interpolate in **v² space**. The ramps are `v² = v₀² + 2ad` — v² is *linear* in distance — so lerping v directly over-accelerates near the start (a chord across a sqrt curve). Lerp the squared velocities, then `Math.sqrt`. Exact on ramps.

### 3.6 **[NEW in v2] Tangential acceleration from the profile**

**Why the finite difference is exact, not an approximation.** Chain rule:
```
dv/dt = (dv/ds)·(ds/dt) = v·(dv/ds)
d(v²)/ds = 2v·(dv/ds) = 2·(dv/dt) = 2a
⇒  a = ½ · d(v²)/ds
```
The profile stores exactly the quantity whose slope this is, **piecewise-linearly in v² space**. So the slope between two samples recovers `a` exactly. Differencing `v` directly would difference a sqrt curve and eat real error. Same v²-space insight that fixed the interpolation bug — reused.

Per segment: `a = (v[i+1]² − v[i]²) / (2·ds)`.

**Sign handles itself:** on the braking ramp `v_f < v₀`, numerator goes negative, `a` comes out negative. No special-casing.

**Output is a staircase, by design.** v² is piecewise-linear, so its derivative is piecewise-*constant* — jumps at sample boundaries, constant within. Do **not** smooth it in the getter; that would invent structure the model doesn't have. Consequence: at ~850 mm/s with step 5 you cross a sample every ~6 ms, faster than the 20 ms loop, so consecutive loops can read different segments and the value jumps. Tolerable in a feedforward; it's why commanded power will look steppy on a scope.

**Real discontinuities at region boundaries.** Leaving a curve-limited region, `a` steps from ~0 to +maxAccel in one segment as the binding constraint switches. Genuine, not a bug. If harsh on hardware, smooth at the *consumer*, not here.

### 3.7 **[NEW in v2] Centripetal acceleration — the resolved form**

`a_c = v_command² · κ · n̂` (mm/s², perpendicular to travel, pointing inward)

- **Magnitude:** `v²κ`, where `v` is the **commanded** speed (`cmdSpeed`, i.e. post-floor), not the raw profile speed. You're commanding `cmdSpeed`, so that's the speed whose inward pull you must supply.
- **Direction:** `normalize(getCentripetalVectorPath(dist))` — **direction only**.

**The units trap (this is the §7 saga, condensed):** `getCentripetalVector` returns the component of `B''` perpendicular to `B'`. Its *direction* is correct (it points toward the center of curvature — verified on a circle: for `r(t)=(cos t, sin t)` at t=0, `B''=(−1,0)` points at the origin). Its *magnitude* is parameter-space garbage — at the arch crest it is **3600**, which is `κ·|B'|²`, a per-unit-t² quantity, **not mm/s²**. Normalizing discards it. That discard is what dissolves the units problem.

**Verified trace (arch crest, t=0.5):**
- `B'(0.5) = (1800, 0)`, `B''(0.5) = (0, −3600)`
- `κ = |1800·(−3600)| / 1800³ = 6,480,000 / 5,832,000,000 = 0.0011111 /mm`
- Curve limit binds: `v = √(800/0.0011111) = 848.5 mm/s`
- `a_c = 848.5² × 0.0011111 = 800 mm/s²`, direction `(0,−1)` → **`(0, −800)`**

**Not a coincidence:** wherever the curve limit is active, `v²κ = (√(a_c/κ))²·κ = a_c` exactly. The profile sized the speed so centripetal load lands exactly at `maxCentripetalAccel`. A built-in consistency check between profile and feedforward.

**Straight sections:** κ→0 so `v²κ→0`, and `normalize` returns `(0,0)` via its zero-guard. Centripetal vanishes cleanly, no divide blowup.

**S-curves:** both the cross-product sign and `getCentripetalVector`'s direction flip at the inflection, so the inward side tracks correctly on its own.

### 3.8 **[NEW in v2] The full acceleration vector**

```
a = tangential + centripetal
```
Perpendicular components. **`a` is a VECTOR; centripetal is one of its two components.** This was a persistent point of confusion — worth restating plainly.

- **Tangential** — along travel. Speeding up / braking. From `getTargetTangentialAcceleration`.
- **Centripetal** — perpendicular, inward. `v²κ`. Keeps you on the curve.

At the arch crest, tangential ≈ 0 (curve-limited, locally flat) so `a` *is* just centripetal — which is probably why it looked like the two were the same thing. At the launch line, κ is small so `a` is nearly all tangential. Mid-curve while still accelerating, both are nonzero and `a` is the diagonal sum.

Both flow through **the same `kA`**. `kA` converts acceleration to power and does not care about the source.

---

## 4. GEOMETRY CODE

### Vector2d.java
```java
package org.firstinspires.ftc.teamcode.odyssey.geometry;

public class Vector2d {
    private final double x, y;

    public Vector2d(double x, double y) { this.x = x; this.y = y; }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getMagnitude() { return Math.hypot(x, y); }

    public Vector2d rotateVector(double theta) {
        double outputX = (x * Math.cos(theta)) - (y * Math.sin(theta));
        double outputY = (x * Math.sin(theta)) + (y * Math.cos(theta));
        return new Vector2d(outputX, outputY);
    }

    public Vector2d subtract(Vector2d other) { return new Vector2d(x - other.getX(), y - other.getY()); }
    public Vector2d add(Vector2d other)      { return new Vector2d(x + other.getX(), y + other.getY()); }
    public Vector2d scale(double factor)     { return new Vector2d(x * factor, y * factor); }

    public double getAngleFromCur() { return Math.atan2(this.y, this.x); }  // atan2(y,x) — order matters

    public double dotProduct(Vector2d other)   { return this.x * other.getX() + this.y * other.getY(); }
    public double crossProduct(Vector2d other) { return this.x * other.getY() - this.y * other.getX(); }

    public Vector2d normalize() {
        double m = getMagnitude();
        if (m == 0) return new Vector2d(0, 0);   // zero-guard — load-bearing for centripetal on straights
        return new Vector2d(x / m, y / m);
    }
}
```
> **Naming nit (not done):** `getAngleFromCur()` → `getAngle()`. Immutable — every op returns a new vector; `v.scale(...)` on its own line does nothing.

### Pose2d.java
```java
package org.firstinspires.ftc.teamcode.odyssey.geometry;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.normalizeAngle;

public class Pose2d {
    private final double x, y, heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x; this.y = y; this.heading = normalizeAngle(heading);
    }
    public Pose2d(Vector2d v, double heading) {
        this.x = v.getX(); this.y = v.getY(); this.heading = normalizeAngle(heading);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public Vector2d getPosition() { return new Vector2d(x, y); }

    // Converts a POINT'S LOCATION field→robot (subtract position, then rotate by −heading)
    public Vector2d toRobotFrame(Vector2d fieldPoint) {
        fieldPoint = fieldPoint.subtract(new Vector2d(x, y));
        return fieldPoint.rotateVector(-heading);
    }
    // Converts a POINT'S LOCATION robot→field
    public Vector2d toFieldFrame(Vector2d robotPoint) {
        robotPoint = robotPoint.rotateVector(heading);
        return robotPoint.add(new Vector2d(x, y));
    }

    public Pose2d relativeTo(Pose2d other) {
        Vector2d otherVec = other.toRobotFrame(this.getPosition());
        double change = normalizeAngle(this.heading - other.heading);
        return new Pose2d(otherVec, change);
    }

    @Override public String toString() { return "Pose2d(x=" + x + ", y=" + y + ", heading=" + heading + ")"; }
}
```
> **CRITICAL distinction:** `toRobotFrame`/`toFieldFrame` convert a **point's location** (they translate). For a **displacement/direction** — a velocity, an **acceleration**, a small motion step — use **`rotateVector` ALONE**. Adding/subtracting position corrupts a direction-like quantity. **[NEW in v2] This is exactly the rule the open Follower bug violates (§6.5).**

### MathUtils.java
```java
// normalizeAngle(angle) = Math.atan2(Math.sin(angle), Math.cos(angle))   → wraps to (−π, π]
// min(a, b)     — 2-arg helper used by VelocityProfile's three passes
// abstractEPS   — small epsilon constant (used by the Laguerre closest-t version)
// max(a,b,c,d)  — ⚠️ [NEW in v2] MecanumDrive calls a 4-ARG max. UNCONFIRMED whether it exists.
//                 If not: nest — Math.max(Math.max(|FL|,|FR|), Math.max(|BL|,|BR|))
```

---

## 5. PATH-PACKAGE CODE

### BezierCurve.java
```java
package org.firstinspires.ftc.teamcode.odyssey.path;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.normalizeAngle;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;

public class BezierCurve {
    private final Vector2d p0, p1, p2, p3;
    private final double startHeading, endHeading;
    private final Vector2d a, b, c, d;

    public BezierCurve(Vector2d p0, Vector2d p1, Vector2d p2, Vector2d p3) {
        this(p0, p1, p2, p3, 0.0, 0.0);   // constructor delegation
    }
    public BezierCurve(Vector2d p0, Vector2d p1, Vector2d p2, Vector2d p3,
                       double startHeading, double endHeading) {
        this.p0 = p0; this.p1 = p1; this.p2 = p2; this.p3 = p3;
        this.startHeading = startHeading; this.endHeading = endHeading;
        this.a = p0.scale(-1).add(p1.scale(3)).subtract(p2.scale(3)).add(p3);
        this.b = p0.scale(3).subtract(p1.scale(6)).add(p2.scale(3));
        this.c = p0.scale(-3).add(p1.scale(3));
        this.d = p0;
    }

    public Vector2d getPoint(double t) { /* Bernstein form */ }
    public Vector2d getTangentVector(double t) { /* B'(t) */ }
    public double   getTangentAngle(double t) { return getTangentVector(t).getAngleFromCur(); }
    public Vector2d getSecondDerivative(double t) { /* B''(t) */ }

    public double getCurvature(double t) {
        Vector2d first = getTangentVector(t), second = getSecondDerivative(t);
        return Math.abs(first.crossProduct(second)) / Math.pow(first.getMagnitude(), 3);
    }

    // component of B'' perpendicular to B'. DIRECTION IS CORRECT; MAGNITUDE IS PARAMETER-SPACE. Normalize it.
    public Vector2d getCentripetalVector(double t) {
        Vector2d v = getTangentVector(t), acc = getSecondDerivative(t);
        return acc.subtract(v.scale(acc.dotProduct(v) / (v.getMagnitude() * v.getMagnitude())));
    }

    public double getLength(double b) {   // arc length t=0 → t=b
        if (b <= 0) return 0;
        UnivariateFunction speed = t -> getTangentVector(t).getMagnitude();
        IterativeLegendreGaussIntegrator integrator = new IterativeLegendreGaussIntegrator(5, 1e-9, 1e-9);
        return integrator.integrate(1000, speed, 0, b);
    }

    public double getTAtDistance(double dist) {   // inverse via Brent
        if (dist <= 0) return 0;
        if (dist >= getLength(1)) return 1;
        UnivariateFunction brent = b -> getLength(b) - dist;
        return new BrentSolver(1e-9, 1e-9).solve(1000, brent, 0, 1);
    }

    public double getHeadingAtT(double t) {   // linear heading interp, shortest-angle
        double delta = normalizeAngle(endHeading - startHeading);
        return normalizeAngle(startHeading + delta * t);
    }

    // CANONICAL closest-point: robust sample-then-refine
    public double getClosestT(Vector2d pose) { /* coarse 0.01 → refine 1e-5, with t=1 fencepost fix */ }
    public double getSquaredDistanceAtT(double t, Vector2d pose) { /* coefficient form + dot(delta,delta) */ }
}
```

### Path.java
```java
public class Path {
    private final BezierCurve[] curves;   // varargs; assumed pre-chained (c.p3 == next.p0)

    public Path(BezierCurve... curves) { this.curves = curves; }

    public double   getTotalLength() { /* sum of curve.getLength(1) */ }
    public Pose2d   getPointOnPath(double distance) { /* walk-and-subtract → Pose2d(point, headingAtT) */ }
    public double   getDistanceOnPath(Vector2d pose) { /* closest curve → cumulative + getLength(t) */ }
    public double   getCurvatureFromPathDistance(double dist) { /* → curve.getCurvature(t) */ }
    public Vector2d getTangentFromPathDistance(double dist)   { /* → curve.getTangentVector(t) */ }
    public Vector2d getCentripetalVectorPath(double dist)     { /* → curve.getCentripetalVector(t) */ }
}
```
> **PERFORMANCE — [ESCALATED in v2] now the sharpest problem in the runtime path.** `getLength(1)` re-runs the integrator every call, and `getTAtDistance` runs Brent which calls `getLength` repeatedly. Every `Path` query is *dozens* of integrations.
>
> **The Follower now makes five heavy path queries per loop** (was four before the accel block; would be seven without the two cheap fixes in §9.2). At 50 Hz this may well **miss the loop budget on hardware**. If the loop runs slow you will see it as **sloppy tracking, not an obvious error** — that is the failure mode to watch for.
>
> **Fix (§9.6):** cache each curve's total length in the `Path` constructor.

### VelocityProfile.java — **[UPDATED in v2]**
```java
package org.firstinspires.ftc.teamcode.odyssey.path;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.min;

public class VelocityProfile {
    private double[] velocities;
    private double[] distances;
    private final double step;

    public VelocityProfile(Path path, double maxVelocity, double maxAcceleration,
                           double maxBrake, double maxCentripetalAccel, double step) {
        this.step = step;
        double totalDistance = path.getTotalLength();
        int arraySize = (int) Math.ceil(totalDistance / step) + 1;   // +1 = fencepost (samples, not gaps)
        distances = new double[arraySize];
        velocities = new double[arraySize];

        // Pass 1 — curve speed limits (local)
        for (int i = 0; i < arraySize; i++) {
            distances[i] = i * step;
            if (distances[i] > totalDistance) distances[i] = totalDistance;   // clamp last sample
            double curvature = path.getCurvatureFromPathDistance(distances[i]);
            double curveLimit = Math.sqrt(maxCentripetalAccel / curvature);
            if (curvature < 1e-4) curveLimit = maxVelocity;   // straight → guard the divide
            velocities[i] = min(maxVelocity, curveLimit);
        }
        // Pass 2 — forward (acceleration)
        velocities[0] = 0;
        for (int i = 1; i < arraySize; i++) {
            double ds = distances[i] - distances[i-1];
            velocities[i] = min(velocities[i],
                Math.sqrt(velocities[i-1]*velocities[i-1] + 2*maxAcceleration*ds));
        }
        // Pass 3 — backward (braking)
        velocities[arraySize - 1] = 0;
        for (int i = arraySize - 2; i >= 0; i--) {
            double ds = distances[i+1] - distances[i];
            velocities[i] = min(velocities[i],
                Math.sqrt(velocities[i+1]*velocities[i+1] + 2*maxBrake*ds));
        }
    }

    public double getTargetVelocity(double distance) {
        if (distances == null || distances.length == 0) return 0;
        int length = distances.length;
        if (distance <= 0) return velocities[0];
        if (distance >= distances[length - 1]) return velocities[length - 1];
        int lowIdx = (int) Math.floor(distance / step);   // FLOOR, not round
        if (lowIdx >= length - 1) lowIdx = length - 2;
        int highIdx = lowIdx + 1;
        double d1 = distances[lowIdx], d2 = distances[highIdx];
        double v1 = velocities[lowIdx], v2 = velocities[highIdx];
        double res = v1*v1 + (distance - d1) * ((v2*v2 - v1*v1) / (d2 - d1));   // lerp in v² space
        return Math.sqrt(res);
    }

    // [NEW in v2] Tangential acceleration. Signed scalar, mm/s². NOT a vector —
    // the Follower supplies direction by scaling the unit tangent by this value.
    public double getTargetTangentialAcceleration(double distance) {
        if (distances == null || distances.length == 0) return 0;
        int length = distances.length;
        double totalDistance = distances[length - 1];

        if (distance >= totalDistance) return 0;   // past the end → no accel command

        int lowIdx = (int) Math.floor(distance / step);
        if (lowIdx < 0) lowIdx = 0;                    // clamp, do NOT early-return 0
        if (lowIdx >= length - 1) lowIdx = length - 2;
        int highIdx = lowIdx + 1;

        double d1 = distances[lowIdx], d2 = distances[highIdx];
        double v0 = velocities[lowIdx];    // EARLIER sample
        double vf = velocities[highIdx];   // LATER sample
        double ds = d2 - d1;
        if (ds <= 1e-9) return 0;          // float near-multiple guard

        return (vf*vf - v0*v0) / (2*ds);   // NOTE THE PARENS
    }
}
```

**Verified values (maxAccel 500, maxBrake 500, step 5):**
| Query | Expected | Why |
|---|---|---|
| distance 0 | **+500** | `v[0]=0`, `v[1]=√(2·500·5)=70.71` → `(5000−0)/10` |
| distance 7 | **+500** | lowIdx 1: `v0=70.71, vf=100, ds=5` → `(10000−5000)/10` |
| cruise | **0** | `v_f = v₀` |
| arch crest | **≈0** | curve-limited, locally flat |
| last segment | **−500** | `v[last]=0` → `(0−5000)/10` |
| past end | **0** | guard |

**Three bugs found in the first draft of this method — all silent, all worth remembering:**
1. **`vf`/`v0` swapped** — labels on the wrong samples negated the whole numerator.
2. **`/ 2*ds` instead of `/ (2*ds)`** — `/` and `*` have equal precedence and evaluate left-to-right, so this computes `(x/2)*ds`, scaling the result by **`ds²`**. Grows worse with coarser `step`.
3. **`if (distance <= 0) return 0; if (distance >= 0) return 0;`** — these two guards cover **every real number**. The method always returned 0 and the entire body was dead code. Compiles clean, no warning. (Only `NaN` reaches the body, since `NaN <= 0` and `NaN >= 0` are both false.)

**Three guards and why each is load-bearing:**
- **Past-the-end** → without it, `lowIdx` clamps to `length−2` forever and you command **−maxBrake indefinitely** after the path ends — a feedforward pushing backward on a stopped robot. Invisible while `kA = 0`; bites the moment `kA` goes live.
- **Low-end clamp (not early-return)** → `Math.floor(−1/5) = −1` → `distances[−1]` → AIOOBE. But returning 0 at `distance ≤ 0` would **kill the launch shove** — distance 0 is exactly where you want +maxAccel. Clamp and fall through.
- **`ds ≤ 1e-9`** → float near-multiples give `ds ≈ 1e-7`, and dividing by that yields a garbage spike (or ±Infinity / NaN) that silently poisons the feedforward. Cheap insurance.

**Effects of adding this method:** none on existing behavior (read-only, no field writes). No visibility changes needed — uses only private fields already in the class (contrast `maxVelocity`, which *is* still unexposed, §9.1). The returned value is exactly the derivative of the curve `getTargetVelocity` reports, since both read the same piecewise-linear v² — **if the interpolation scheme ever changes, both must change together.**

---

## 6. CONTROL / LOCALIZATION / FOLLOWER / DRIVE CODE

### 6.1 PIDController.java (adapted from Charles Grassin's MIT PID — KEEP ATTRIBUTION)
```java
// ⚠️ Restore the MIT license/copyright header (Charles Grassin) at the top of this file,
//    and credit it in the project README. Required for the open-source library.

public class PIDController {
    private double setPoint, kP, kI, kD;
    private double minLimit = Double.NaN, maxLimit = Double.NaN;
    private double previousTime = Double.NaN;
    private double lastError = 0, integralError = 0;
    private double maxIntegral = Double.NaN;   // windup clamp, in OUTPUT units, opt-in

    public double getOutput(final double currentTime, final double currentValue) {
        final double error = setPoint - currentValue;
        final double dt = !Double.isNaN(previousTime) ? (currentTime - previousTime) : 0;
        final double derivativeError = (dt != 0) ? ((error - lastError) / dt) : 0;
        integralError += error * dt;
        if (!Double.isNaN(maxIntegral) && kI != 0) {
            double contrib = kI * integralError;
            if (contrib > maxIntegral)       integralError = maxIntegral / kI;
            else if (contrib < -maxIntegral) integralError = -maxIntegral / kI;
        }
        previousTime = currentTime;
        lastError = error;
        return checkLimits((kP * error) + (kI * integralError) + (kD * derivativeError));
    }

    public void reset() { previousTime = Double.NaN; lastError = 0; integralError = 0; }  // NEW path
    private void resetErrors() { lastError = 0; integralError = 0; }                      // gain changes
    // checkLimits, setOutputLimits, removeOutputLimits, setMaxIntegral(abs), getters, setters
}
```
> `dt`-aware (I × dt, D ÷ dt) — more correct than MiniPID's per-loop model. First run handled via `previousTime = NaN`. Split reset: gain changes keep timing; path restart wipes it. **Open nit:** `setSetpoint` still calls full `reset()`; arguably should call `resetErrors()`.

### 6.2 Localizer.java
```java
public interface Localizer {
    void update();
    Pose2d getPose();
}
```

### 6.3 **[NEW in v2] PinpointLocalizer.java** (TeamCode) — BUILT, VERIFIED

Implements `Localizer`. Wraps `GoBildaPinpointDriver`. Its entire job is translating FTC's **`Pose2D`** (capital D, `com.qualcomm...`) into odyssey's **`Pose2d`** (lowercase d).

- `update()` → `driver.update()` (pulls a fresh I²C reading)
- `getPose()` → `getPosX(DistanceUnit.MM)`, `getPosY(DistanceUnit.MM)`, `getHeading(AngleUnit.RADIANS)` → `new Pose2d(x, y, heading)`

**Use the unit-explicit getters, not the bare `getPosition()`.** Odyssey is mm and radians everywhere; demanding those units at the hardware boundary is how you dodge a units bug. Heading normalization is free — the `Pose2d` constructor wraps it.

**Reset:** `resetPosAndIMU()` is Pinpoint-specific, so it is **not** on the `Localizer` interface (core must not know about hardware). Two clean options:
- If `PinpointLocalizer` builds the driver internally: add a public `resetPosAndIMU()` that delegates. **In the OpMode, declare the field as `PinpointLocalizer`, not `Localizer`** — the interface type only exposes two methods. The Follower still takes it as a `Localizer`; polymorphism is unaffected.
- If it takes a driver instance: reset the driver in the OpMode before passing it in.

**Reset rules:** call it **once**, in `init()`/`start()`, never in `loop()` (it blocks ~0.25 s). **Robot must be physically still** — it recalibrates the IMU; jostling gives a bad heading zero that skews everything after. It zeroes **both** position and heading, so wherever the robot sits becomes (0,0,0). Related: `recalibrateIMU()` (IMU only, keeps position), `setPosition(Pose2D)` (start at a known non-zero field pose — what a real auto uses).

**Other driver methods:** `getPosition()`, `setOffsets()`, `setEncoderResolution(GoBildaOdometryPods.goBILDA_SWINGARM_POD)`, `setEncoderDirections()`.

### 6.4 **[UPDATED in v2] DriveSignal.java** — now 5-arg, carries acceleration
```java
package org.firstinspires.ftc.teamcode.odyssey.follower;

public class DriveSignal {
    private final double forwardVelocity;      // mm/s,   robot frame
    private final double strafeVelocity;       // mm/s,   robot frame
    private final double forwardAcceleration;  // mm/s²,  robot frame
    private final double strafeAcceleration;   // mm/s²,  robot frame
    private final double turn;                 // rad/s   ← NOT mm/s

    public DriveSignal(double forwardVelocity, double strafeVelocity,
                       double forwardAcceleration, double strafeAcceleration, double turn) {
        this.forwardVelocity = forwardVelocity;
        this.strafeVelocity = strafeVelocity;
        this.turn = turn;
        this.forwardAcceleration = forwardAcceleration;
        this.strafeAcceleration = strafeAcceleration;
    }

    public double getForwardVelocity()     { return forwardVelocity; }
    public double getStrafeVelocity()      { return strafeVelocity; }
    public double getTurn()                { return turn; }
    public double getForwardAcceleration() { return forwardAcceleration; }
    public double getStrafeAcceleration()  { return strafeAcceleration; }
}
```

**Constructor param order is `(f, s, af, as, turn)` — turn LAST.** Verified against the Follower's call site; they match. This matters because all five params are `double`, so a mismatch would compile silently and put `turn` into a strafe-accel field.

**[BREAKING] Getters were renamed** (`getForward` → `getForwardVelocity`, `getStrafe` → `getStrafeVelocity`) and **the 3-arg constructor was removed.** Consequences:
- **`SimulatedLocalizer.applyDriveSignal` calls the old names → `odyssey-core` will not compile until fixed.** This also breaks `FollowerSimTest`.
- Any other 3-arg construction site breaks.
- The renames are *better* names — keep them; just fix the call sites in one pass rather than one compile error at a time.

**Why it goes through `DriveSignal` at all:** `MecanumDrive` has no path, no profile, no pose — it cannot compute `a` itself. Only the Follower has the ingredients. A `drive(signal, accel)` two-arg alternative works but splits one command across two params that must always travel together.

**No angular-acceleration field** — heading is pure PID with no feedforward, so `α` has no producer and no consumer. Per the "wait for a caller" rule.

**Mixed units now.** Two fields mm/s, two mm/s², one rad/s. The names carry it; a class comment stating units would help. **The v1 comment "all three are VELOCITIES (mm/s)" was already wrong about `turn` and is now doubly wrong.** `turn` being rad/s is load-bearing: `MecanumDrive` multiplies it by `k` (mm) to get mm/s.

**The sim ignores the accel fields entirely** — `applyDriveSignal` reads only velocity and turn. Once the renames are fixed, sim behavior is bit-for-bit identical. **Which also means the sim can never validate the accel path** (§8.2).

### 6.5 **[UPDATED in v2] Follower.java** — 🟡 accel block added, ROTATION BUG OPEN
```java
public class Follower {
    private final Path path;
    private final Localizer localizer;
    private final VelocityProfile velocityProfile;
    private final PIDController pidTranslational, pidHeading;
    // ⏳ should become: private final double minDriveSpeed, floorCutoff;
    // ⏳ should become: private final double totalLength;   (constant — hoist to constructor)

    public DriveSignal update(double currentTime) {
        Pose2d pose = localizer.getPose();
        double distance = path.getDistanceOnPath(pose.getPosition());
        Pose2d reference = path.getPointOnPath(distance);
        double totalLength = path.getTotalLength();              // ⏳ hoist to constructor
        double distanceRemaining = totalLength - distance;
        double profileSpeed = velocityProfile.getTargetVelocity(distance);
        double floor = 110;                                      // ⏳ hardcoded — make a ctor param
        double cmdSpeed = (distanceRemaining > 500)              // ⏳ hardcoded — make a ctor param
                          ? Math.max(profileSpeed, floor) : profileSpeed;

        // 1) TRANSLATIONAL (corrective)
        Vector2d offset = reference.getPosition().subtract(pose.getPosition());
        double pidOut = pidTranslational.getOutput(currentTime, -offset.getMagnitude());
        Vector2d translationalVector = offset.normalize().scale(pidOut);

        // 2) DRIVE (along path)
        Vector2d tangentUnit = path.getTangentFromPathDistance(distance).normalize();
        Vector2d driveVector = tangentUnit.scale(cmdSpeed);

        // 3) HEADING PID
        double diff = normalizeAngle(reference.getHeading() - pose.getHeading());
        double heading = pidHeading.getOutput(currentTime, -diff);

        // 4) VELOCITY SUM → robot frame
        Vector2d sum = translationalVector.add(driveVector);
        Vector2d velocityVector = sum.rotateVector(-pose.getHeading());

        // 5) ACCELERATION VECTOR  [NEW in v2]
        Vector2d centripetalUnit = path.getCentripetalVectorPath(distance).normalize();
        double curvature = path.getCurvatureFromPathDistance(distance);
        Vector2d centripetalVector = centripetalUnit.scale(cmdSpeed * cmdSpeed * curvature);

        double profileAccel = velocityProfile.getTargetTangentialAcceleration(distance);
        Vector2d tangentialVector = tangentUnit.scale(profileAccel);   // ⏳ reuse tangentUnit, don't re-query

        Vector2d accelVector = centripetalVector.add(tangentialVector);
        // ❌ OPEN BUG — accelVector is in FIELD frame and is never rotated:
        // Vector2d accelRobot = accelVector.rotateVector(-pose.getHeading());

        return new DriveSignal(velocityVector.getX(), velocityVector.getY(),
                               accelVector.getX(), accelVector.getY(), heading);
    }
}
```

#### ❌ **THE OPEN BUG — accel vector never rotated to robot frame**

The velocity sum gets `.rotateVector(-pose.getHeading())`. The accel vector does **not**, and goes into `DriveSignal` in **field frame**.

**Traced at the start of the arch (heading = 90°):**
- `B'(0) = (0, 1800)`, `B''(0) = (7200, −3600)` → `κ(0) = 12,960,000 / 1800³ = 0.0022222 /mm`
- Centripetal perp component = `(7200, 0)` → unit `(1, 0)` (inward = +x, since the arch bends right)
- Magnitude at `cmdSpeed = 110`: `110² × 0.0022222 = 26.9`
- Tangential = `+500` along unit tangent `(0,1)` → `(0, 500)`
- **Field-frame sum: `(26.9, 500)`**
- Rotated by −90°: **`(500, −26.9)`** ← correct. Forward accel 500, small rightward strafe accel.
- **As written it passes `(26.9, 500)`** — forward and strafe components swapped. `kA` would command a massive *sideways* shove at the launch line instead of a forward one.

**Why it's nasty: at the arch crest, heading = 0, so field and robot frames coincide and the bug is invisible.** It only manifests where heading ≠ 0.

**Fix — one line, and it must be `rotateVector` ALONE.** Acceleration is direction-like, not a location; `toRobotFrame` would corrupt it by subtracting the robot's position (§4, §13).

#### Other Follower notes

- **PID sign convention:** both PIDs use `setPoint = 0` and are fed the **negated** measured error (`-offset.getMagnitude()`, `-diff`) so output pushes toward the target. Keep consistent or the robot drives/turns the wrong way.
- **`update` takes `currentTime`** (elapsed seconds). The Follower has no clock; the caller supplies it.
- **Dead constructor param `double factor`** — taken, never assigned. Presumably the start of making the floor configurable. Finish it or drop it.
- **Floor edge case:** `distanceRemaining > 500` means **on any path shorter than 500 mm the floor never engages** — straight back to the §8 self-start deadlock. The arch (~1680 mm) is fine, but a short path silently reverts to broken. Making the cutoff a param fixes it.
- **Floor numbers are well chosen for the arch.** Floor 110 binds only in the first ~12 mm (`v²/2a = 110²/1000 = 12.1`). During braking it would only bind inside the last 12 mm, which is already past the 500 mm cutoff — so **floor and braking never fight.** Verified.
- **Using `cmdSpeed` (not `profileSpeed`) for the centripetal magnitude is correct** — you're commanding `cmdSpeed`, so that's the speed whose inward pull you must supply. Using `profileSpeed` would under-command during the floor.
- **Floor/accel disagreement at launch:** while the floor holds speed flat at 110, the profile still reports +500 tangential accel. Commanded velocity isn't actually changing, so the feedforward briefly overstates. Harmless, but it's why launch may kick slightly. **Once `kA` is live you may be able to lower the floor** — the two attack the same launch problem from different directions and they stack.
- **Braking sign:** past the crest, tangential accel goes negative, `tangentialVector` points backward along the path, and `kA·a` **subtracts** power. Correct — that's the clean stop. But `kS·signum(v)` still shoves forward, so the two oppose in the last few millimetres. Expect slight twitchiness at the stop.
- **Unused imports:** `SimulatedLocalizer`, `BezierCurve`, `MathUtils`. The first matters most — importing the sim class into the Follower is exactly the coupling to avoid, even unused.

### 6.6 **[NEW in v2] MecanumDrive.java** (TeamCode) — 🟡 written, constructor config MISSING

```java
package org.firstinspires.ftc.teamcode.odyssey.drive;

public class MecanumDrive {
    private final DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private final double kS, kV, kA, k;
    private final VoltageSensor voltageSensor;
    // ⏳ consider: private final double lateralMultiplier;   (§6.7)

    public MecanumDrive(double kS, double kV, double kA, double lX, double lY,
                        DcMotorEx leftFront, DcMotorEx rightFront,
                        DcMotorEx leftBack, DcMotorEx rightBack, VoltageSensor voltageSensor) {
        /* assignments; this.k = lX + lY; */
        // ❌ MISSING — see below
    }

    public void drive(DriveSignal signal) {
        double f = signal.getForwardVelocity();
        double s = signal.getStrafeVelocity();
        double w = signal.getTurn();

        double v_FL = f - s - k*w;
        double v_FR = f + s + k*w;
        double v_BL = f + s - k*w;
        double v_BR = f - s + k*w;

        double af = signal.getForwardAcceleration();
        double as = signal.getStrafeAcceleration();

        double a_FL = af - as;    // no k·α term — no angular-accel field, by design
        double a_FR = af + as;
        double a_BL = af + as;
        double a_BR = af - as;

        double voltageComp = 12.0 / voltageSensor.getVoltage();

        double p_FL = (kS*Math.signum(v_FL) + kV*v_FL + kA*a_FL) * voltageComp;
        double p_FR = (kS*Math.signum(v_FR) + kV*v_FR + kA*a_FR) * voltageComp;
        double p_BL = (kS*Math.signum(v_BL) + kV*v_BL + kA*a_BL) * voltageComp;
        double p_BR = (kS*Math.signum(v_BR) + kV*v_BR + kA*a_BR) * voltageComp;

        double max = max(Math.abs(p_FL), Math.abs(p_FR), Math.abs(p_BL), Math.abs(p_BR));
        if (max > 1.0) { p_FL /= max; p_FR /= max; p_BL /= max; p_BR /= max; }

        leftFront.setPower(p_FL);  rightFront.setPower(p_FR);
        leftBack.setPower(p_BL);   rightBack.setPower(p_BR);
    }
}
```

#### ❌ **BLOCKER — the constructor never configures the motors**

Motors are stored but never have mode, direction, or zero-power behavior set. **Motor state lives on the controller and persists between OpModes** — `hardwareMap.get` hands you the existing object, it does not reset anything. Three distinct effects:

1. **Mode.** If a motor is left in `RUN_USING_ENCODER` (e.g. by `testr1`), `setPower` stops meaning "voltage fraction" and starts meaning "fraction of max velocity," with the hub's PID regulating underneath — **double-regulating the feedforward and making kS/kV meaningless.** Worse: on dead-wheel odometry the drive encoders may not be plugged in, leaving the hub with no feedback. **Set `RUN_WITHOUT_ENCODER` explicitly** — the math is only valid there, so this class should assert it rather than trust the caller.
2. **Direction.** All four default to `FORWARD`, but two are physically mirror-mounted, so all-four-positive makes the robot **spin in place instead of driving forward** — every equation correct, robot doing the wrong thing. `setDirection(REVERSE)` on the mirrored side cancels the mirroring, which is what makes "all four positive = forward" true.
3. **Zero power behavior.** Default is `FLOAT` — motors coast at zero power, so the robot **drifts past the endpoint**, directly undercutting the millimetre accuracy the library exists for. Set `BRAKE`.

#### The four equations, explained

Each wheel's speed is the sum of three contributions:
- **`f` (forward)** — plus on all four. All wheels same direction, same speed.
- **`s` (strafe)** — sign pattern down the column is `− + + −`. **Diagonal pairs spin opposite ways** (FL & BR one way, FR & BL the other); the 45° rollers convert that opposition into pure sideways motion.
- **`k·ω` (turn)** — sign pattern `− + − +`. **Left side vs right side**, tank-style.

**Wheel direction comes out of the arithmetic** — negative wheel velocity → negative power → `setPower` runs that motor backward. No per-wheel special-casing is needed or wanted.

**`k = l_x + l_y`** converts rad/s to mm/s: `l_x` = half the wheelbase (front-to-back, centre to centre), `l_y` = half the track (left-to-right). Units: mm × rad/s = mm/s, which is what lets `k·ω` sit in the same equation as `f` and `s`. Both terms because mecanum turns using the rollers as well as the wheels. Typical FTC: **300–350 mm**. Measure the real robot; if you get 50 or 900, something is off.

**Traced — straight ahead** (`f=500, s=0, ω=0`, kV=0.00064, kS=0.1, kA=0, 12.5 V):
all wheels 500 mm/s → `(0.1 + 0.32) × 0.96 =` **0.403** on all four. Sanity: 500 mm/s is ~32% of 1568 top speed, commanding ~40% power; the extra is kS overhead. ✓

**Traced — saturating turn** (`f=1350, s=0, ω=1, k=330`, 12.0 V):
`v = 1020, 1680, 1020, 1680` → `p = 0.753, 1.175, 0.753, 1.175` → max 1.175 → **0.641, 1.0, 0.641, 1.0**. Ratios preserved. ✓

**Traced — pure strafe** (`f=0, ω=0, s=1000`): `FL=−1000, FR=+1000, BL=+1000, BR=−1000`. Diagonal pattern. ✓

#### Normalization — why divide all four

`setPower` accepts −1..1 and **silently clips** anything beyond. Clipping truncates *one* wheel and leaves the others, changing the **ratios** between wheels — and the ratios determine which direction the robot goes.

- **Clipping** the saturating-turn case → `0.753, 1.0, 0.753, 1.0`: left/right difference shrinks from 0.422 to 0.247, so the robot turns far less than commanded. Wrong motion.
- **Normalizing** → `0.641, 1.0, 0.641, 1.0`: ratios unchanged, same direction, ~85% speed.

**Rule: when you can't have everything you asked for, give up *speed*, not *direction*.** A robot on the right path running slower is recoverable — the Follower sees it lag. A robot going the wrong direction is a tracking error the PID must fight.

- **Use `Math.abs`** — a wheel at −1.4 is as saturated as +1.4. Maxing raw values would return 0.5 for `{0.5, −1.4, 0.3, −0.9}`, skip normalization entirely, and let the SDK clip −1.4. (Correct result for that case: divide by 1.4 → `0.357, −1.0, 0.214, −0.643`.)
- **Only when `max > 1.0`** — dividing unconditionally would *amplify* a command the Follower never asked for.
- **`MathUtils.min` is unrelated** — that's the 2-arg helper for the profile's three passes. There is no lower-bound normalization step; the lower bound is stiction, handled by `kS`.

#### Effects and interactions

- **Normalization weakens turn authority at saturation** — it scales the turn contribution down with everything else, so heading correction gets quieter exactly when the robot is working hardest. If heading tracking degrades at high speed, this is why. Fix is lowering the profile's `maxVelocity`, not touching this code.
- **Normalization only *approximately* preserves direction, because of `kS`.** Scaling power by `c` does not scale velocity by `c`: solving `c·(kS + kV·v)` back gives `v' = c·v + kS(c−1)/kV` — an **additive** offset per wheel, not proportional. In the saturating trace that's ≈ −23 mm/s each, shifting the FL/FR ratio from 0.607 to 0.601. Negligible, universal to every static-feedforward FTC drivetrain, but real.
- **`kS` fights braking near zero speed** — `kA·a` subtracts while `kS·signum(v)` still shoves forward. Expect slight twitchiness in the last few millimetres.
- **`kS` has no deadband** — at v = 1 mm/s you still command a full 0.1 shove. If the robot jitters when it should be still, add a small deadband (`|v| < ~5 mm/s → power = 0`).
- **Voltage comp is applied to the whole power including `kS`** — correct; stiction needs a voltage, and power is a fraction of battery voltage. **Applied before normalization** — also correct, since comp can push past 1.0.
- **Voltage comp stabilizes tuning.** Without it, kV effectively drifts ~13% across a match as the battery sags (13 V fresh → 11.5 V tired) and you'd re-tune chasing a moving target.
- **Roller mounting is physical, not code.** Viewed from above, the top rollers must form an **X**. Mounted wrong, strafe fails no matter what the code says — the robot crabs diagonally or goes nowhere.
- **Bench check for strafe: watch the wheels, not the robot.** You should see the diagonal pattern. All four spinning the same way ⇒ a `setDirection` is wrong. Right pattern but no sideways motion ⇒ roller mounting.
- **`kA` is whatever the caller passes.** Pass **0** until the signs are verified — accel terms compute and get multiplied by zero, so you can validate the kinematics without the feedforward confusing the picture.
- **No `stop()` method.** An all-zero signal gives zero power on all four (`signum(0) = 0`, so no kS shove), so one isn't strictly needed. Worth confirming on the bench that motors actually cut when the OpMode ends rather than holding their last power.
- **⚠️ `max(a,b,c,d)` is 4-arg.** `MathUtils` per v1 has only 2-arg `min`; `Math.max` is 2-arg. **Confirm this exists or it won't compile.**
- **Nothing calls `drive()` yet** — inert until `FollowerAuto` exists.

### 6.7 **[NEW in v2] The lateral (strafe) multiplier**

**Why strafe is slower, and where the loss actually is.** Check the kinematics: pure strafe `s=1000` gives wheel speeds `−1000, +1000, +1000, −1000`; pure forward `f=1000` gives `+1000` on all four. **Same magnitudes.** Geometrically, strafing should be exactly as fast as driving forward.

It isn't, in reality — the loss is in **roller contact**: rollers scrub sideways, small-diameter rollers have high rolling resistance, and load goes through roller bearings. Real drivetrains land around **80–90%** of forward.

**Therefore the correction does NOT belong per-wheel.** `kV` is a property of a motor and wheel — a wheel just spins; it doesn't know whether the robot is going forward or sideways. And every wheel velocity is a mix of f, s, and ω, so no wheel is "the strafe one."

**The fix — a lateral multiplier, one `final` constructor param, applied to the input before the equations:**
```java
s  = signal.getStrafeVelocity()     * lateralMultiplier;
as = signal.getStrafeAcceleration() * lateralMultiplier;
```
**Both** — velocity and feedforward must agree, same rule as sign flips.

Value **> 1**: if commanding 1000 yields 800 measured, `L = 1000/800 = 1.25`. **Default `1.0` = no correction**, so adding the param changes nothing until tuned.

**Measure it** (localization is verified, so this is now possible): command pure strafe at a known speed for a known time, read Pinpoint displacement, `L = commanded / actual`.

**How much it matters on the arch:** at t=0.25 the tangent is `atan2(900, 1350) = 33.7°` while the linearly-interpolated heading is 45°. That ~11° gap puts ~19% of the drive vector into strafe. Small but real. **It matters far more on a heading-locked path** (robot facing one way through a whole curve) — then strafe dominates.

**Effects:** scaling `s` up makes powers larger, so strafe-heavy motion **normalizes more often**, scaling the whole command down. If `L` is wrong the robot under/over-strafes and the translational PID catches it reactively — you get drift-then-correct wobble rather than clean tracking. **`kS` is unaffected** — stiction is per-wheel and per-direction-of-spin, not per-robot-direction.

**No turn multiplier.** Physically the same story (real rotation scrubs all four wheels, so `k·ω` under-delivers), but `turn` comes from the heading PID, which is **feedback** — error drives more output. **Honest caveat:** with `kI = 0` you have P-only, so systematic under-delivery leaves a **residual steady-state heading error**. That's a tuning fix (add `kI`, or raise `kP`), not a kinematics one.

### 6.8 SimulatedLocalizer.java — ❌ BROKEN, needs the getter renames
```java
public class SimulatedLocalizer implements Localizer {
    private Pose2d pose;
    public SimulatedLocalizer(Pose2d startPose) { this.pose = startPose; }   // REQUIRED — else null
    @Override public Pose2d getPose() { return this.pose; }
    @Override public void update() { }

    // KINEMATIC, MASSLESS model — pose += velocity·dt. No inertia, no slip.
    public Pose2d applyDriveSignal(DriveSignal signal, double deltaTime) {
        double forwardDistance = signal.getForward() * deltaTime;   // ❌ → getForwardVelocity()
        double strafeDistance  = signal.getStrafe()  * deltaTime;   // ❌ → getStrafeVelocity()
        double dHeading        = signal.getTurn()    * deltaTime;
        Vector2d robotFrameStep = new Vector2d(forwardDistance, strafeDistance);  // x=forward, y=strafe
        Vector2d fieldStep = robotFrameStep.rotateVector(pose.getHeading());      // rotate ONLY
        pose = new Pose2d(pose.getPosition().add(fieldStep), dHeading + pose.getHeading());
        return pose;
    }
}
```
> **Axis convention (must stay consistent everywhere):** robot-frame vector is `(x = forward, y = strafe)`, and `rotateVector(+heading)` maps robot→field. The Follower's inverse is `rotateVector(−heading)`. This is why `+strafe = left` is the assumed convention in the mecanum equations.

---

## 7. THE CENTRIPETAL SITUATION — **[RESOLVED in v2]**

This was the hardest conceptual knot in the project. **It is now settled.** Restating the resolution and the reasoning, so it is never re-litigated:

- **`v²·κ` is an ACCELERATION** (mm/s²). The Follower's drive/translational vectors are **VELOCITIES** (mm/s). **You cannot add an acceleration to a velocity sum** — unit error. This is why the earlier attempt broke tracking: at the arch crest, `v²κ ≈ 800` was roughly equal to the drive vector and pointed perpendicular, commanding the robot to dive off the arch at ~43°.
- **In the massless, velocity-commanded sim, there is genuinely nothing for a centripetal term to correct** — no inertia, no outward drift. The two-vector follower (translational + drive) is correct *for the sim*.
- **On the real robot, centripetal is essential.** Mass → inertia → outward drift on curves. **It is not optional, and it is not adequately handled by the translational PID alone.** The PID is *reactive*: the robot drifts wide, then gets pulled back. Feedforward is *preventive*: the inward pull is commanded before drift happens. **Feedforward is the goal; reactive is the floor.** This distinction is what separates a good follower from a merely adequate one.
- **It enters as an acceleration feedforward through `kA`**, exactly as Road Runner does. Pedro uses a tuned `centripetalScaling` constant that launders the units empirically; Odyssey's is derived.
- **Dead end to avoid:** an AI-generated "centripetal velocity" formula (`v_c = (a_c × v_⊥)/|v_⊥|²`, added to the translational vector). Dimensionally inconsistent, and conceptually wrong — there is **no such thing as centripetal velocity.** Following a circle means velocity stays *tangent* while *acceleration* points inward. Adding an inward velocity component would spiral the robot *into* the curve.

**Implementation status:** the math is fully specified and verified (§3.7), the Follower computes it (§6.5), `DriveSignal` carries it (§6.4), and `MecanumDrive` applies it through `kA` (§6.6). **The only things left are the open rotation bug and measuring `kA`.**

> **⚠️ A caution for the next assistant, learned the hard way this session.** When `setVelocity` came up, it was briefly and wrongly implied that centripetal might not be needed because the translational PID handles the drift. **That is wrong and the person correctly rejected it.** Centripetal is a key part of a good pathing system; the only question was ever *which mechanism* implements it, and the answer is the `kA` feedforward. Do not soften this again.

---

## 8. CURRENT STATE

### 8.1 Simulation
`FollowerSimTest` runs the Follower against `SimulatedLocalizer` (`update(t)` → `applyDriveSignal(signal, 0.02)` → `t += 0.02`), then asserts the final pose.

Test path = a **symmetric arch**: `BezierCurve((0,0),(0,600),(1200,600),(1200,0), 90°, −90°)`.
Profile: `(maxVel 1000, maxAccel 500, maxBrake 500, maxCentripetal 800, step 5)`.
PIDs: `kP=1, kI=0, kD=0`; translational clamp ±1000, heading clamp ±3.

**Arc length ≈ 1680 mm** (Simpson estimate on `|B'(t)| = |(7200t(1−t), 1800(1−2t))|` with n=4; the "~1.2 m" in v1 was the straight-line displacement, not arc length).

**Profile shape on this path:** reaching maxVel 1000 would need 1000 mm of accel and 1000 mm of braking = 2000 mm > 1680, so the profile is **triangular, never reaching maxVelocity.** The unconstrained peak would be ~917 mm/s at the midpoint, but **the crest curve limit of 848.5 binds first.** Verified.

**[NEW in v2] Curvature is NOT maximal at the crest.** κ(0.5) = 0.001111, but κ(0.25) ≈ 0.001896 — *higher*. So the tightest curve limit (~650 mm/s) is around **t ≈ 0.25 and 0.75**, not the crest. The crest is a local *maximum* of the curve limit. It still binds (the accel ramp from t=0.25 would reach ~927 there), and the profile is locally flat at the crest, so **tangential accel ≈ 0 at the crest holds.** (Simpson-based estimate — worth confirming numerically if it ever matters.)

**Test status: ❌ ALL BROKEN — will not compile** until `SimulatedLocalizer` is updated for the renamed `DriveSignal` getters. Before that break, the results were:
- ✅ `reachesTheEndOfThePath` — starts `(0, 0.5, 90°)`, ends x≈1200. **PASSED.**
- ✅ `staysNearThePath` — starts `(0, 0.5, 90°)`, worst drift < 100 mm. **PASSED.**
- ❌ `recoversFromABadStart` — starts `(-100, -200, 0°)`, expects x≈1200, ends x≈0. **FAILED.**

**Why that failure is the SELF-START DEADLOCK, not a tracking bug:** from `(-100,-200)` the translational vector correctly homes the robot to the path **start** (distance ≈ 0). At distance 0 the profile commands **0 speed** (`velocities[0]=0`). Zero speed → no motion → still distance 0 → still 0 speed → **stuck forever.** The two passing tests escape only because they start **0.5 mm along** the path.

The `(0, 0.5)` nudge is defensible for the sim (the sim is unrealistically perfect — massless, frictionless, exact — which is what creates the fixed point; real sensor noise breaks it instantly). But it means the follower **cannot self-start from a true standstill**, which is a real defect. **The floor in §6.5 is the fix; it needs its params made configurable, and the test re-run once the sim compiles again.**

### 8.2 **[NEW in v2] What the sim can and cannot validate**

**The sim can never validate the acceleration path.** `SimulatedLocalizer` is massless and kinematic (`pose += velocity·dt`), so it has **no inertia and cannot respond to an acceleration command at all.** Feeding it a correct `a` and a garbage `a` produce **identical motion**. This is the same reason removing centripetal broke nothing in the sim.

Therefore:
- ✅ You **can** unit-test the *numbers* — assert `getTargetTangentialAcceleration` returns +500 at distance 0, that the accel vector is `(500, −26.9)` at the start and `(0, −800)` at the crest.
- ❌ You **cannot** test that the feedforward *helps*. That only shows up on hardware.

### 8.3 Hardware
- ✅ **`PinpointLocalizer` built.**
- ✅ **PUSH TEST RUN AND PASSED** — localization is verified truthful. This was the single highest-value action available and it is done. Everything downstream now means something.
- ⏳ `MecanumDrive` written but not configured, not wired, not bench-verified.
- ⏳ `FollowerAuto` does not exist — nothing calls `drive()`.
- ⏳ No robot constants measured yet (`kS`, real `kV`, `kA`, `lX`, `lY`, `lateralMultiplier`).

---

## 9. OPEN PROBLEMS / NEXT STEPS (priority order)

### Blocking — must be done before anything drives

1. **❌ Rotate the accel vector into robot frame** (`Follower`, §6.5). One line. A real bug producing plausible-looking wrong numbers. **Fix this before running any static-analysis sweep** — no linter will catch it, since it's a frame-convention error, not a syntax one.
2. **❌ Configure the motors in `MecanumDrive`'s constructor** (§6.6) — `RUN_WITHOUT_ENCODER`, `BRAKE`, `setDirection` on the mirrored side. Absence of a method call is legal Java; no tool will flag it.
3. **❌ Fix `SimulatedLocalizer`** for the renamed `DriveSignal` getters — `odyssey-core` does not compile until this is done, which also blocks every sim test.
4. **⚠️ Confirm `MathUtils` has a 4-arg `max`** — or nest `Math.max` in `MecanumDrive`.
5. **⏳ Build `FollowerAuto`** (§9.1) — the missing top of the stack.

### Then — bench work (now unblocked by the passing push test)

6. **Measure `lX`, `lY`** — tape measure, wheel centre to wheel centre, halve each. `k = lX + lY`.
7. **Find `kS`** — command a small constant power to all four, raise until the wheels break loose. Expect **0.05–0.15**.
8. **Verify signs, one command at a time**, watching Pinpoint telemetry: pure forward (X rises), pure strafe (Y moves, X doesn't, and *watch the wheels* for the diagonal pattern), pure turn (heading rises for +ω). **If strafe is backwards, flip `s` in all four velocity equations AND `as` in all four accel equations — together.**
9. **Measure real `kV`** — drive flat out, log actual top speed. `kV = 1 / (measured top speed)`.
10. **Measure `lateralMultiplier`** (§6.7) — pure strafe, known speed and time, `L = commanded / actual`.
11. **Tune the PIDs** — `kP` only first; add `kD` when it oscillates; `kI` last (but note §6.7's steady-state heading caveat).

### Then — the last switch

12. **Measure `kA` and turn centripetal on.** Drive at a known constant acceleration, log commanded power vs achieved acceleration, solve `kA = (power − kS − kV·v) / a`. Set it nonzero. **Centripetal goes live at that moment** — everything upstream is already built.

### Known, deferred, not blocking

13. **Length caching in `Path`** (§5) — **escalated**. Five heavy path queries per Follower loop, each running Brent over Gauss-Legendre. Likely to miss the 50 Hz budget on hardware, and it manifests as sloppy tracking rather than an obvious error. Cache each curve's total length in the `Path` constructor.
14. **Two cheap Follower wins** — reuse the single tangent query instead of calling `getTangentFromPathDistance` twice; hoist `getTotalLength()` into the constructor (it never changes). Seven heavy queries → five.
15. **Make the floor configurable** — `minDriveSpeed` and `floorCutoff` as `final` ctor params, replacing the hardcoded 110/500 and the dead `factor` param. **Also fixes the short-path deadlock** (§6.5). Note `maxVelocity` is still swallowed inside `VelocityProfile` with no getter, so deriving the floor as a fraction of it still isn't possible without exposing it.
16. **Add a path-complete check** — `FollowerAuto` doesn't self-terminate.
17. **Restore the PID MIT header (Charles Grassin) + README attribution.**
18. **`PathBuilder`** (fluent `.startAt().splineTo().build()`); GUI JSON export/import (a `PathData` DTO in `odyssey-core` shared by GUI-write and robot-read, via Gson); exact-Laguerre closest-t as an optional high-precision mode; naming cleanups (`getAngleFromCur` → `getAngle`; the `pose` param in `BezierCurve` that's actually a `Vector2d` position); import cleanup; `MecanumDrive` math split into core (§2).

### 9.1 **[NEW in v2] `FollowerAuto` — spec**

The OpMode that runs everything. `@Autonomous`, extends `OpMode`. Nothing moves without it.

**`init()` — build once:**
- Four `DcMotorEx` from `hardwareMap` by config name; voltage sensor via `hardwareMap.voltageSensor.iterator().next()`
- `PinpointLocalizer`; call `resetPosAndIMU()` (robot still)
- `BezierCurve`s → `Path`
- `VelocityProfile(path, maxVel, maxAccel, maxBrake, maxCentripetal, step)`
- Both `PIDController`s + output limits
- `MecanumDrive(kS, kV, kA=0, lX, lY, ...motors, voltageSensor)`
- `Follower(path, localizer, profile, pidTrans, pidHeading, minDriveSpeed, floorCutoff)`

**`start()`:** create and reset an `ElapsedTime`. This is the clock the Follower doesn't have.

**`loop()` — four lines:**
```
localizer.update();
DriveSignal signal = follower.update(timer.seconds());
mecanumDrive.drive(signal);
telemetry: pose, distance along path, commanded speed
```

**Traps:**
- **`localizer.update()` must come first.** Skip it and the pose never refreshes — the Follower runs on a frozen position and drives off confidently. Same ordering trap as the push test.
- **`timer.seconds()` must be monotonic elapsed seconds.** The PIDs compute `dt` from consecutive calls; a reset mid-run makes `dt` wrong or negative and the D term explodes. Reset once in `start()`, never again.
- **Nothing stops the robot at the end.** The profile commands ~0 and BRAKE holds it, but `loop()` keeps running and the Follower keeps computing. Add a `distance ≥ totalLength` check eventually.
- **Starting pose must match the path's start.** `resetPosAndIMU` makes wherever the robot sits (0,0,0). If the path starts at (0,0) heading 90°, the robot must physically be there facing that way — or use `setPosition` to tell the Pinpoint where it really is. Mismatch ⇒ the Follower thinks it's off-path immediately and lunges.
- **`init()` runs on the INIT press**, so the expensive `VelocityProfile` construction (all that Gauss-Legendre work) happens there rather than in `loop()`. Good.

---

## 10. HARDWARE STATUS & REMAINING ORDER OF OPERATIONS

**✅ Step 1 — `PinpointLocalizer` built.** Done.
**✅ Step 2 — PUSH TEST PASSED.** Localization verified. This was the gate on everything; it's cleared.

**The push test, for reference** (no follower, no motors, no driving): print the Pinpoint pose to telemetry and move the robot by hand.
- Push exactly **1 m forward** → X reads ~1000 mm, Y stays ~0
- Push **sideways** a known distance → Y tracks, X stays put
- Rotate **90°** → heading reads ~π/2 (≈1.571 rad)

Failure → fix: wrong axis moves ⇒ X/Y swapped. Reads ~700 or ~1300 for 1000 ⇒ encoder resolution. Reads negative ⇒ encoder direction flipped. Heading goes the wrong way ⇒ heading direction flipped.

**⏳ Step 3 — `MecanumDrive` + `FollowerAuto`.** Fix the constructor config; wire it up. `kA = 0`.
**⏳ Step 4 — measure the real constants.** `lX`, `lY`, `kS`, `kV`, `lateralMultiplier`. Don't trust the §11 estimates.
**⏳ Step 5 — run the follower, tune the PIDs.** `kP` only, then `kD`, then `kI`.
**⏳ Step 6 — measure `kA`; centripetal goes live.**

**Be honest with the person:** a clean, accurate path-follow will not happen the first session on the drive layer. First contact surfaces sign errors, unmeasured constants, and tuning all at once. A realistic good outcome for the next session is: robot *moving* under the follower with correct signs, `kS`/`kV` roughly measured, and tuning started.

---

## 11. FTC ROBOT CONSTANTS — **[CORRECTED in v2]**

For a ~13.5 kg robot, **96 mm goBILDA mecanum wheels**, four **312 RPM goBILDA Yellow Jacket** motors, on FTC foam tiles. **AI-estimated and arithmetic-checked, NOT measured.** Do NOT profile at the traction limit; run at ~50–70% — a slipping mecanum wheel destroys odometry. Units: mm, mm/s².

| Constant | Theoretical / limit | Suggested profile value |
|---|---|---|
| Max forward velocity | ~1568 mm/s (`π·96·312/60`, verified) | ~1350 mm/s |
| Max strafe velocity | **see correction below** | handled by `lateralMultiplier`, not a separate cap |
| Max acceleration | ~6870 mm/s² (`μg`, μ≈0.7 — upper bound) | ~3000–4000 mm/s² |
| Max braking | ~6870 mm/s² | ~3000–4000 mm/s² |
| Max centripetal accel | ~4900 mm/s² (lower lateral μ) | ~2500–3000 mm/s² |
| Profile `step` | — | 5 mm |
| `kV` (theoretical) | `1/1568 ≈ 0.00064` | start here; real is ~15–25% higher under load |
| `kS` | not derivable — pure stiction | measure; expect 0.05–0.15 |
| `kA` | not derivable | must be measured |
| `k = lX + lY` | — | measure; typical FTC 300–350 mm |
| `lateralMultiplier` | — | 1.0 until measured; expect ~1.1–1.25 |

> **[CORRECTED in v2] The v1 "max strafe = 1568 × 1/√2 = 1108" does NOT follow from the kinematics.** Traced: pure strafe and pure forward produce **identical wheel-speed magnitudes** — there is no √2 anywhere in the mecanum equations. The real strafe loss is roller scrub and roller rolling resistance (~80–90% of forward), which is **empirical, not derived**, and is handled by `lateralMultiplier` (§6.7) rather than a separate velocity cap.

> **What is and isn't derivable** (asked directly this session): **`kV` is derivable** and fine as a starting value — free speed is unloaded so real kV runs ~15–25% higher, and the PID absorbs that. **`kA` is not really derivable** — it needs effective inertia (not just 13.5 kg; wheels and gearboxes spin too), gearbox efficiency, and position on the torque-speed curve; each is a guess and they multiply. **`kS` is not derivable at all** — it's stiction: gearbox drag, wheel scrub, belt tension. Only the robot can tell you.

> **The one thing that could never be guessed** is whether the robot *knows where it is*. That's binary, not tunable, and no kV/PID/kA value can fix or even reveal a lying pose. That was the push test's whole job — and it passed.

---

## 12. KEY DECISIONS LOG (so they aren't re-litigated)

- **Cubic Bézier** (not quadratic): independent start/end heading control, S-curves, superset of lines/quadratics.
- **Arc-length parameterization with Brent inverse** (Gauss-Legendre for length): the core precision feature; the integral has no closed form so it must be numerical. This is what Pedro lacks.
- **Drive-vector follower** (corrective + drive + heading), **not pure pursuit** (no lookahead-target chasing).
- **Closest-point = sample-then-refine** (robust) as canonical, exact-Laguerre optional; robustness chosen because closest-point feeds an approximate PID and localization noise dwarfs the precision difference.
- **PID: adapted from C. Grassin's MIT `dt`-aware PID** (keep attribution), plus output clamp and integral windup clamp; split reset.
- **Velocity profile: distance-indexed, 3-pass, interpolated in v² space.** Enabled by arc length; Pedro can't do this.
- **Centripetal: an ACCELERATION, delivered as `kA` feedforward — NOT a velocity, NOT summed into the velocity vector, and NOT optional.** (§7 — hard-won, twice.)
- **[NEW in v2] Tangential acceleration derived analytically as `½·d(v²)/ds`** — exact on ramps, falls out of the v²-space representation already chosen.
- **[NEW in v2] Open-loop `setPower` + feedforward (`kS + kV·v + kA·a`), not `setVelocity`.** `setVelocity` works and the hub's PIDF would handle velocity→power (kV becomes the F coefficient), but **it takes a velocity and nothing else — there is no argument for an acceleration feedforward.** Centripetal would have to be laundered as a fudge on the commanded velocity. `setPower` + feedforward is Road Runner's model, is what §9.3 always specced, and gives `kA` a proper home. Also moot here: dead-wheel odometry means the drive encoders may not even be wired.
- **[NEW in v2] Voltage compensation from the start** (`×12.0/V_batt`) — this is the one thing `setVelocity` was doing for free. Costs one line and removes a whole class of "why is it slower in match 4."
- **[NEW in v2] `MecanumDrive` takes motors injected, not a `HardwareMap`** — same instinct as `Follower` taking a `Localizer`. Config names stay in the OpMode; the class stops caring where dependencies came from; mock motors become possible in tests. Use the same style consistently for `PinpointLocalizer`.
- **[NEW in v2] Strafe correction is a single input-side multiplier, not per-wheel** — the loss is roller physics at the robot level, and `kV` is a per-motor property. (§6.7)
- **[NEW in v2] Normalization divides all four powers, never clips one** — give up speed, not direction. (§6.6)
- **Multi-module Gradle restructure** so the library is standalone/reusable and a JavaFX GUI can depend on it; `odyssey-core` kept pure-Java.
- **GUI: JavaFX desktop app** reusing the real library. Draws the FTC field + draggable control points; mm↔px with Y-flip via `FieldCoordinates`. JSON export deferred.

---

## 13. RECURRING GOTCHAS

- **`Vector2d` is immutable** — `v.scale(x)` / `v.normalize()` on their own line do nothing; **assign the result.**
- **Displacement vs. location:** rotate a *displacement/direction* — velocity, **acceleration**, a motion step — with `rotateVector` ALONE. Only a *point's location* uses `toFieldFrame`/`toRobotFrame`. **[v2] This is the open Follower bug.**
- **[NEW in v2] Every direction-like quantity in a `DriveSignal` must be rotated to robot frame.** If you add a new vector output, ask immediately: which frame is it in? The velocity sum is rotated; the accel vector was not, and the omission is invisible wherever heading ≈ 0.
- **`Math.atan2(y, x)`** — y first, x second. Backwards mirrors every heading across the diagonal.
- **`!= NaN` never works in Java** — always `Double.isNaN(x)`.
- **`double` is a primitive** — unassigned it's `0.0`, never `null`, so no NPE, just a silently wrong value.
- **[NEW in v2] Local variables shadow fields silently.** `double ticksPerMM = ...` inside a method declares a *new local* and leaves the field at `0.0`. Compiles clean; the motor just never spins. (Bit the `testr1` OpMode.)
- **[NEW in v2] `/` and `*` have EQUAL precedence and evaluate left to right.** `x / 2*ds` is `(x/2)*ds`, **not** `x/(2*ds)`. Parenthesize denominators. This one scaled a result by `ds²`.
- **[NEW in v2] Check that guard clauses don't cover the whole domain.** `if (d <= 0) return 0; if (d >= 0) return 0;` covers every real number — the method body became dead code with no warning. Read guards as a set, not one at a time.
- **[NEW in v2] Keep `v₀`, `v_f`, and `ds` on the SAME segment.** Mixing indices gives a plausible number from the wrong place — no crash, no NaN, just a feedforward shifted one segment early.
- **[NEW in v2] Renaming a public getter breaks every caller silently until compile.** Grep for old names across all modules in one pass.
- **[NEW in v2] Motor state persists between OpModes.** `hardwareMap.get` returns the existing object; mode, direction, and zero-power behavior carry over from whatever ran last. Set them explicitly every time.
- **[NEW in v2] `Math.abs` before taking the max when normalizing** — a wheel at −1.4 saturates as hard as +1.4.
- **Fencepost:** N samples span N−1 gaps; `count = length/step + 1`.
- **Interpolate in the RIGHT space** — velocity ramps are linear in v², not v.
- **Guard divides:** `normalize` at zero magnitude, curvature at κ≈0, PID at dt=0, accel getter at ds≈0.
- **Don't build speculative methods** — wait for a caller.
- **Verify before asserting** — units, arithmetic, a traced example.

---

## 14. **[NEW in v2] CORRECTIONS TO v1**

Two v1 claims were traced this session and found wrong. Both are fixed in the body above; recorded here so the error isn't reintroduced.

1. **§5's "exact multiple ⇒ last two samples coincide" is impossible.** With `n = ceil(totalDistance/step)` and `arraySize = n+1`, the sample at index `n−1` is always strictly below `totalDistance`, so it never clamps; only the final sample can. Example: `totalDistance = 100, step = 5` → `arraySize = 21`, `distances[19] = 95`, `distances[20] = 100`. No coincidence. **The real risk is float near-multiples** (`totalDistance = 100.0000001` → `ds ≈ 1e-7` → garbage spike), which is why the `ds ≤ 1e-9` guard stays.

2. **§11's "max strafe = 1568 × 1/√2" does not follow from the kinematics.** Pure strafe and pure forward give identical wheel-speed magnitudes; there is no √2 in the mecanum equations. The real loss is roller scrub, it's empirical (~80–90%), and it belongs in a `lateralMultiplier`. (§6.7, §11)

**Also worth recording — an assistant error this session, since §0 exists to prevent exactly this.** When `setVelocity` was introduced, it was briefly implied that centripetal might be unnecessary because the translational PID handles curve drift reactively. **That was wrong, it contradicted this very document, and the person correctly and forcefully rejected it.** The correct framing, never to be softened again: centripetal is essential to a good pathing system; the only open question was ever *which mechanism* implements it; the answer is the `kA` acceleration feedforward; and it is scheduled last purely because of dependency order, never because it's optional.

---

*End of handoff v2. The push test has passed, so the foundation is trustworthy. The single most valuable next action is fixing the two blockers — the unrotated accel vector in `Follower` and the missing motor configuration in `MecanumDrive`'s constructor — because neither will be caught by a compiler or a static-analysis sweep.*