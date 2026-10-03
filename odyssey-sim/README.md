# odyssey-sim

Give it an OpMode `.java` file; it compiles it, runs it on a simulated robot, and shows where the
robot went compared to the path.

```
./gradlew :odyssey-sim:run --args="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odyssey/opmodes/FirstTest.java"
./gradlew :odyssey-sim:test
```

The path is relative to the repo root. Each run prints a summary and writes
`odyssey-sim/out/<OpMode>/`:

- `report.html`: the path the OpMode built, where the robot really went, and where the Pinpoint
  thought it was, drawn on the field. Also speed, wheel powers and battery voltage over time, plus
  the telemetry.
- `trace.csv`: the same data every 10 ms.

Options: `--time <s>` sets how long to run after START (30 for `@Autonomous`, 10 otherwise).
`--set key=value` changes the robot (see below). `--out <dir>` changes where the files go.

## Without a computer set up for the robot

`.github/workflows/simulate.yml` runs the simulator on GitHub's machines on every push:
1. It runs the tests, then simulates each OpMode in `TeamCode/.../odyssey/opmodes/`.
2. Open the repo's **Actions** tab and click the run.
3. The summary page shows each OpMode's results, and the full `report.html` / `trace.csv` files
   are under **Artifacts** as `simulator-reports`.

To try a new auto, add its file to that folder and push, or create it on github.com.

Once the workflow is on `main`, **Run workflow** on the Actions tab can also simulate one file by path.

## What the file can use

The file is compiled against:
- stand-ins for the FTC SDK classes OpModes use (`OpMode`, `LinearOpMode`, `HardwareMap`,
  `DcMotorEx`, `VoltageSensor`, `ElapsedTime`, `Telemetry`, units, both Pinpoint drivers);
- `odyssey-core`;
- TeamCode's `odyssey` package (`MecanumDrive`, `PinpointLocalizer`, ...).

Anything else, like a camera or another class of yours, doesn't compile, and the error says so.

The simulated robot has the drive motors `leftFront`, `rightFront`, `leftBack` and `rightBack`, a
Pinpoint called `localizer`, and the hub's voltage sensor. Asking `hardwareMap` for any other name
fails the way a missing device does on the robot.

## The simulated robot

- **Drivetrain.** A 14 kg mecanum chassis. Each wheel is a DC motor with back-EMF, BRAKE/FLOAT at
  zero power, and static + rolling friction. Strafing loses some force. The battery sags under
  load, and the hub's voltage reading shows the sag.
- **Calibrated to this robot.** The defaults give the kS / kV / kA measured with
  `StaticCoefficientOpMode` / `kAOpMode`. Each run prints the simulated robot's constants.
- **Motors act like the SDK's.**
  - Each motor's physical mounting is modeled, so a wrong `setDirection` drives the wrong way.
  - An unchanged power isn't re-sent.
  - `setPower(NaN)` arrives as 0 and is flagged.
  - RUN_TO_POSITION and `setVelocity` (the hub's PIDs) aren't simulated and throw.
- **Pinpoint.** Odometry with a little noise and heading drift. `resetPosAndIMU()` and `setPos*` work
  as on the robot. Pod setup calls are accepted, but the simulated pods are always set up right.
- **Time.** Simulated time moves when the OpMode talks to hardware:
  - about 1.8 ms per motor write or voltage read, and 2.5 ms per `pinpoint.update()`;
  - about 1 ms of SDK work per `loop()`;
  - `sleep()` / `idle()` / `opModeIsActive()` in a `LinearOpMode`.

  So the loop rate comes from what the code does: `FirstTest` runs at 80 Hz. Runs are repeatable.
- **Field.** 12 ft square with walls, robot starting at the center facing +X. Change that with
  `start.*`.

Every setting, with where the default comes from, is in
[`src/main/resources/odysseysim/robot.properties`](src/main/resources/odysseysim/robot.properties):

```
./gradlew :odyssey-sim:run --args="path/to/MyAuto.java --set robot.massKg=16 --set battery.openCircuitVolts=12.2"
```

## Limits

- No wheel slip, no collisions except with the walls, and nobody on the gamepads.
- A `LinearOpMode` that busy-waits without hardware calls, `sleep()` or `opModeIsActive()` (e.g. on an
  `ElapsedTime`) never lets time move. The simulator notices after 10 s and shows where it's stuck.
- The physics and the I/O costs are approximations. The simulator is for catching bugs and
  comparing changes; it doesn't replace tuning on the real robot.
