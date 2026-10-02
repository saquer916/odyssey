package org.firstinspires.ftc.teamcode.odysseysim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

// Runs one OpMode against the simulated robot.
//
// Time only moves when the OpMode does something that takes time on a real robot: a bus
// transaction (setPower, getVoltage, pinpoint.update(), ...), the SDK's work between loop() calls,
// or sleep()/idle()/opModeIsActive() in a LinearOpMode. The physics integrates in small steps
// across each interval, so a motor command lands when its write finishes and a read sees the robot
// as of that moment. The loop rate falls out of what the OpMode actually does.
public final class Simulation {
    private static final String[] WHEEL_KEYS = {"leftFront", "rightFront", "leftBack", "rightBack"};

    private static Simulation current;

    public static Simulation current() {
        if (current == null) throw new IllegalStateException("odyssey-sim: no simulation is running");
        return current;
    }

    final DrivePhysics physics;
    final SimPinpoint pinpoint;
    final SimMotor[] wheels = new SimMotor[4];
    final HardwareMap hardwareMap = new HardwareMap();
    final SimTelemetry telemetry;
    final Trace trace = new Trace();

    final double motorWriteMs;
    final double motorReadMs;
    final double voltageReadMs;
    final double pinpointUpdateMs;
    final double pinpointWriteMs;
    private final double physicsStep;
    private final double recordInterval;
    private final double loopOverhead;
    private final double idleTime;
    private final double initSeconds;
    private final double stopGrace;
    private final double settleSeconds;

    private final Set<String> warnings = new LinkedHashSet<>();
    private final Map<String, Integer> nanPowerCounts = new LinkedHashMap<>();
    final List<Double> loopPeriodsMs = new ArrayList<>();

    private double time;
    private double nextSample;
    private double startTime = Double.NaN;
    private double endTime = Double.POSITIVE_INFINITY;
    private double stopTime = Double.NaN;
    private double runSeconds;
    private double hardStopTime = Double.POSITIVE_INFINITY;
    private boolean started;
    private boolean stopRequested;
    private String phase = "init";
    private OpMode opMode;
    private double lastActiveCheck = Double.NaN;
    private Throwable failure;

    public Simulation(SimConfig config) {
        Random seeds = new Random((long) config.number("sim.seed"));
        physics = new DrivePhysics(config);
        pinpoint = new SimPinpoint(config, new Random(seeds.nextLong()));

        motorWriteMs = config.number("io.motorWriteMs");
        motorReadMs = config.number("io.motorReadMs");
        voltageReadMs = config.number("io.voltageReadMs");
        pinpointUpdateMs = config.number("io.pinpointUpdateMs");
        pinpointWriteMs = config.number("io.pinpointWriteMs");
        physicsStep = config.number("timing.physicsStepMs") / 1000.0;
        recordInterval = config.number("sim.recordIntervalMs") / 1000.0;
        loopOverhead = config.number("timing.loopOverheadMs") / 1000.0;
        idleTime = config.number("timing.idleMs") / 1000.0;
        initSeconds = config.number("sim.initSeconds");
        stopGrace = config.number("sim.stopGraceSeconds");
        settleSeconds = config.number("sim.settleSeconds");

        for (int i = 0; i < 4; i++) {
            double mount = config.number("mount." + WHEEL_KEYS[i]);
            if (mount != 1 && mount != -1) {
                throw new IllegalArgumentException("odyssey-sim: mount." + WHEEL_KEYS[i] + " must be 1 or -1");
            }
            wheels[i] = new SimMotor(this, i, config.string("hw." + WHEEL_KEYS[i]), mount, config);
            hardwareMap.put(wheels[i].name, wheels[i]);
        }
        hardwareMap.put(config.string("hw.voltageSensor"), new SimVoltageSensor(this, new Random(seeds.nextLong())));
        hardwareMap.putFactory(config.string("hw.pinpoint"), Simulation::createPinpointDriver);
        telemetry = new SimTelemetry(this);

        current = this;
        SimClock.set(0);
    }

    private static Object createPinpointDriver(Class<?> requested) {
        if (!SimPinpointDriver.class.isAssignableFrom(requested) || Modifier.isAbstract(requested.getModifiers())) {
            return null;
        }
        try {
            return requested.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("odyssey-sim: can't create " + requested.getName(), e);
        }
    }

    // ---- running ---------------------------------------------------------------------------------

    public void run(Class<? extends OpMode> type, double runSeconds) {
        this.runSeconds = runSeconds;
        trace.record(this);
        nextSample = recordInterval;
        try {
            opMode = type.getDeclaredConstructor().newInstance();
        } catch (InvocationTargetException e) {
            failure = e.getCause();
        } catch (ReflectiveOperationException e) {
            failure = e;
        }
        if (opMode != null) {
            opMode.hardwareMap = hardwareMap;
            opMode.telemetry = telemetry;
            try {
                if (opMode instanceof LinearOpMode) {
                    ((LinearOpMode) opMode).runOpMode();
                } else {
                    runIterative(opMode);
                }
            } catch (SimStop e) {
                warn(String.format("the OpMode was still running %.1f s after stop was requested; "
                        + "the robot controller would have force-stopped it", stopGrace));
            } catch (Throwable t) {
                failure = t;
            }
        }
        finish();
    }

    private void runIterative(OpMode op) {
        op.init();
        afterCall();
        while (!started) {
            if (time >= initSeconds) {
                markStarted();
                break;
            }
            if (op.simStopRequested()) {
                requestStop();
                op.stop();
                return;
            }
            op.init_loop();
            afterCall();
        }
        op.start();
        afterCall();
        while (true) {
            checkStop();
            if (stopRequested) break;
            double loopStart = time;
            op.time = op.getRuntime();
            op.loop();
            afterCall();
            loopPeriodsMs.add((time - loopStart) * 1000.0);
        }
        op.stop();
        telemetry.autoUpdate();
    }

    private void afterCall() {
        telemetry.autoUpdate();
        advance(loopOverhead);
    }

    private void markStarted() {
        started = true;
        startTime = time;
        endTime = time + runSeconds;
        phase = "run";
    }

    private void requestStop() {
        if (stopRequested) return;
        stopRequested = true;
        stopTime = time;
        phase = "stop";
        hardStopTime = time + stopGrace;
    }

    private void checkStop() {
        if (started && time >= endTime) requestStop();
        if (opMode != null && opMode.simStopRequested()) requestStop();
    }

    // The SDK sets every motor to 0 when the OpMode ends; keep simulating while the robot stops.
    private void finish() {
        requestStop();
        hardStopTime = Double.POSITIVE_INFINITY;
        for (SimMotor wheel : wheels) wheel.forceStop();
        phase = "stopped";
        advance(settleSeconds);
    }

    // ---- LinearOpMode -----------------------------------------------------------------------------

    public void waitForStart() {
        if (started || stopRequested) return;
        advance(initSeconds - time);
        markStarted();
    }

    public boolean opModeIsActive() {
        advance(loopOverhead);
        if (started && !stopRequested) {
            if (!Double.isNaN(lastActiveCheck)) loopPeriodsMs.add((time - lastActiveCheck) * 1000.0);
            lastActiveCheck = time;
        }
        checkStop();
        return started && !stopRequested;
    }

    public boolean opModeInInit() {
        advance(loopOverhead);
        if (!started && time >= initSeconds) markStarted();
        checkStop();
        return !started && !stopRequested;
    }

    public boolean isStarted() {
        if (!started) {
            advance(idleTime);
            if (time >= initSeconds) markStarted();
        }
        return started;
    }

    public boolean isStopRequested() {
        advance(idleTime);
        checkStop();
        return stopRequested;
    }

    public void idle() {
        advance(idleTime);
        checkStop();
    }

    public void sleep(double seconds) {
        advance(seconds);
        checkStop();
    }

    // ---- time ---------------------------------------------------------------------------------------

    void io(double milliseconds) {
        advance(milliseconds / 1000.0);
    }

    void advance(double seconds) {
        if (!(seconds > 0)) return;
        double[] power = new double[4];
        boolean[] braking = new boolean[4];
        for (int i = 0; i < 4; i++) {
            power[i] = wheels[i].wheelPower();
            braking[i] = wheels[i].braking();
        }
        double target = time + seconds;
        while (time < target - 1e-12) {
            double dt = Math.min(physicsStep, target - time);
            physics.step(dt, power, braking, time);
            pinpoint.onPhysicsStep(dt, physics.vx, physics.vy, physics.omega);
            time += dt;
            SimClock.set(time);
            if (time >= nextSample - 1e-9) {
                trace.record(this);
                nextSample += recordInterval;
            }
        }
        if (time > hardStopTime) {
            hardStopTime = Double.POSITIVE_INFINITY;
            throw new SimStop();
        }
    }

    double time() {
        return time;
    }

    double timeSinceStart() {
        return started ? time - startTime : time - initSeconds;
    }

    String phase() {
        return phase;
    }

    // Field pose (mm, rad) of the Pinpoint's origin: the frame the OpMode's coordinates are in.
    double[] pinpointOrigin() {
        double oh = physics.heading - pinpoint.idealH;
        double c = Math.cos(oh);
        double s = Math.sin(oh);
        return new double[] {
                physics.x * 1000.0 - (c * pinpoint.idealX - s * pinpoint.idealY),
                physics.y * 1000.0 - (s * pinpoint.idealX + c * pinpoint.idealY),
                oh
        };
    }

    // ---- results ------------------------------------------------------------------------------------

    void warn(String message) {
        if (warnings.add(message)) System.out.println("  ! " + message);
    }

    void recordNaNPower(String motor) {
        Integer count = nanPowerCounts.get(motor);
        nanPowerCounts.put(motor, count == null ? 1 : count + 1);
        warn("setPower(NaN) on \"" + motor + "\": the hub ends up applying 0 power");
    }

    OpMode opMode() { return opMode; }
    Throwable failure() { return failure; }
    boolean started() { return started; }
    double startTime() { return startTime; }
    double stopTime() { return stopTime; }
    double initSeconds() { return initSeconds; }
    Set<String> warnings() { return warnings; }
    Map<String, Integer> nanPowerCounts() { return nanPowerCounts; }

    List<String> telemetryLog() {
        List<String> lines = new ArrayList<>();
        for (SimTelemetry.Entry e : telemetry.logEntries) lines.add(e.text);
        return lines;
    }
}
