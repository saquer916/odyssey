package org.firstinspires.ftc.teamcode.odysseysim;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Collections;

public class SimulatorTest {
    // Gradle runs tests from the odyssey-sim folder.
    private static final String OPMODES = "../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odyssey/opmodes/";

    // The constants measured on the real robot, which the default simulated robot is built from.
    private static final double KS = 0.11265261;
    private static final double KV = 0.000444;
    private static final double KA = 0.000154;

    @Rule public TemporaryFolder tmp = new TemporaryFolder();

    private Class<? extends OpMode> compile(String file) throws IOException {
        return OpModeCompiler.compile(new File(file), tmp.newFolder());
    }

    private static Simulation run(Class<? extends OpMode> type, double seconds, String... settings) {
        SimConfig config = SimConfig.defaults();
        for (String s : settings) config.set(s);
        Simulation sim = new Simulation(config);
        sim.run(type, seconds);
        return sim;
    }

    private static Trace.Sample sampleAt(Simulation sim, double time) {
        Trace.Sample best = null;
        for (Trace.Sample s : sim.trace.samples) {
            if (best == null || Math.abs(s.time - time) < Math.abs(best.time - time)) best = s;
        }
        return best;
    }

    private static boolean logged(Simulation sim, String text) {
        for (String line : sim.telemetryLog()) if (line.contains(text)) return true;
        return false;
    }

    // Drives straight forward at a fixed power from START; left side reversed like the tuning OpModes.
    public static class ConstantPower extends OpMode {
        static double power = 0.5;
        private DcMotorEx[] motors;

        @Override
        public void init() {
            String[] names = {"leftFront", "rightFront", "leftBack", "rightBack"};
            motors = new DcMotorEx[4];
            for (int i = 0; i < 4; i++) motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
            motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public void start() {
            for (DcMotorEx m : motors) m.setPower(power);
        }

        @Override
        public void loop() {}
    }

    public static class SendsNaN extends ConstantPower {
        @Override
        public void loop() {
            hardwareMap.get(DcMotorEx.class, "leftFront").setPower(Double.NaN);
        }
    }

    public static class WrongMotorName extends OpMode {
        @Override public void init() { hardwareMap.get(DcMotorEx.class, "frontLeft"); }
        @Override public void loop() {}
    }

    public static class LinearForward extends LinearOpMode {
        @Override
        public void runOpMode() {
            DcMotorEx[] m = {
                    hardwareMap.get(DcMotorEx.class, "leftFront"), hardwareMap.get(DcMotorEx.class, "rightFront"),
                    hardwareMap.get(DcMotorEx.class, "leftBack"), hardwareMap.get(DcMotorEx.class, "rightBack")};
            m[0].setDirection(DcMotorSimple.Direction.REVERSE);
            m[2].setDirection(DcMotorSimple.Direction.REVERSE);
            waitForStart();
            for (DcMotorEx motor : m) motor.setPower(0.5);
            sleep(1000);
            for (DcMotorEx motor : m) motor.setPower(0);
            while (opModeIsActive()) {
                idle();
            }
        }
    }


    @Test public void firstTestReachesTheEndOfItsPath() throws IOException {
        Simulation sim = run(compile(OPMODES + "FirstTest.java"), 3);
        Report report = new Report(sim, "FirstTest");
        assertNull(sim.failure());
        assertTrue(sim.nanPowerCounts().isEmpty());
        assertEquals(1, report.paths.size());
        assertTrue("ended " + report.endError + " mm from the end", report.endError < 10);
        assertTrue("strayed " + report.maxCrossTrack + " mm from the path", report.maxCrossTrack < 10);
    }

    @Test public void loopTimeComesFromHardwareCalls() throws IOException {
        // FirstTest's loop: pinpoint update 2.5 + voltage read 1.8 + 4 motor writes 1.8 + SDK 1.0 ms
        Simulation sim = run(compile(OPMODES + "FirstTest.java"), 1);
        assertEquals(12.5, sim.loopPeriodsMs.get(sim.loopPeriodsMs.size() / 2), 0.01);
    }

    @Test public void driveSignCheckPasses() throws IOException {
        Simulation sim = run(compile(OPMODES + "DriveSignCheck.java"), 6);
        assertNull(sim.failure());
        assertTrue(logged(sim, "FORWARD PASS"));
        assertTrue(logged(sim, "STRAFE_LEFT PASS"));
        assertTrue(logged(sim, "TURN_CCW PASS"));
    }

    @Test public void driveSignCheckCatchesMirroredMotors() throws IOException {
        // A chassis where the right side is the mirrored one, as main's MecanumDrive assumed.
        Simulation sim = run(compile(OPMODES + "DriveSignCheck.java"), 6,
                "mount.leftFront=1", "mount.leftBack=1", "mount.rightFront=-1", "mount.rightBack=-1");
        assertTrue(logged(sim, "FORWARD FAIL"));
    }

    @Test public void steadySpeedMatchesKsAndKv() {
        ConstantPower.power = 0.5;
        // 3 s at ~870 mm/s would hit the wall from the field center
        Simulation sim = run(ConstantPower.class, 3, "battery.openCircuitVolts=12", "battery.internalOhms=0", "field.walls=false");
        double expected = (0.5 - KS) / KV;
        assertEquals(expected, sampleAt(sim, 2.9).speed, expected * 0.01);
    }

    @Test public void timeConstantMatchesKaOverKv() {
        ConstantPower.power = 0.5;
        Simulation sim = run(ConstantPower.class, 1, "battery.openCircuitVolts=12", "battery.internalOhms=0");
        double steady = (0.5 - KS) / KV;
        assertEquals(0.632 * steady, sampleAt(sim, KA / KV).speed, steady * 0.03);
    }

    @Test public void powerBelowKsDoesNotMoveTheRobot() {
        ConstantPower.power = 0.1;
        Simulation sim = run(ConstantPower.class, 2);
        Trace.Sample last = sim.trace.samples.get(sim.trace.samples.size() - 1);
        assertTrue(Math.abs(last.trueX) < 1.0);
    }

    @Test public void nanPowerArrivesAsZero() {
        ConstantPower.power = 0.0;
        Simulation sim = run(SendsNaN.class, 1);
        assertTrue(sim.nanPowerCounts().get("leftFront") > 0);
        Trace.Sample last = sim.trace.samples.get(sim.trace.samples.size() - 1);
        assertTrue(Math.abs(last.trueX) < 1.0);
    }

    @Test public void unknownDeviceNameFailsLikeTheSdk() {
        Simulation sim = run(WrongMotorName.class, 1);
        assertTrue(sim.failure() instanceof IllegalArgumentException);
        assertTrue(sim.failure().getMessage().contains("Unable to find a hardware device with name \"frontLeft\""));
    }

    @Test public void linearOpModesRun() {
        Simulation sim = run(LinearForward.class, 3);
        assertNull(sim.failure());
        Trace.Sample last = sim.trace.samples.get(sim.trace.samples.size() - 1);
        assertTrue(last.trueX > 300);
        assertEquals("stopped", last.phase);
    }

    @Test public void compileErrorsAreReported() throws IOException {
        File broken = tmp.newFile("Broken.java");
        Files.write(broken.toPath(), Collections.singletonList(
                "public class Broken extends com.qualcomm.robotcore.eventloop.opmode.OpMode { int x = ; }"), StandardCharsets.UTF_8);
        try {
            OpModeCompiler.compile(broken, tmp.newFolder());
            fail("expected a compile error");
        } catch (IllegalArgumentException e) {
            assertTrue(e.getMessage().contains("Broken.java doesn't compile"));
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void settingTyposAreRejected() {
        SimConfig.defaults().set("robot.mass=12");
    }
}
