package org.firstinspires.ftc.teamcode.odysseysim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

// ./gradlew :odyssey-sim:run --args="path/to/MyAuto.java"
public final class SimMain {
    private static final String USAGE = String.join("\n",
            "usage: ./gradlew :odyssey-sim:run --args=\"<OpMode.java> [options]\"",
            "",
            "  <OpMode.java>       the OpMode to simulate (path from the repo root)",
            "  --time <seconds>    how long to run after START (default 30 for @Autonomous, 10 otherwise)",
            "  --set key=value     change the simulated robot, e.g. --set robot.massKg=16 (repeatable;",
            "                      the settings are in odyssey-sim/src/main/resources/odysseysim/robot.properties)",
            "  --out <dir>         where to write report.html and trace.csv (default odyssey-sim/out/<OpMode>)");

    private SimMain() {}

    public static void main(String[] args) throws Exception {
        File source = null;
        Double seconds = null;
        File out = null;
        List<String> sets = new ArrayList<>();
        for (int i = 0; i < args.length; i++) {
            String a = args[i];
            if (a.equals("--time")) seconds = Double.parseDouble(value(args, ++i, a));
            else if (a.equals("--set")) sets.add(value(args, ++i, a));
            else if (a.equals("--out")) out = new File(value(args, ++i, a));
            else if (a.equals("-h") || a.equals("--help")) {
                System.out.println(USAGE);
                return;
            } else if (a.startsWith("--") || source != null) fail("unexpected argument \"" + a + "\"");
            else source = new File(a);
        }
        if (source == null) fail("pass the OpMode .java file to simulate");

        String name = source.getName().replaceAll("\\.java$", "");
        if (out == null) out = new File(new File("odyssey-sim").isDirectory() ? "odyssey-sim/out" : "sim-out", name);

        Class<? extends OpMode> type;
        SimConfig config = SimConfig.defaults();
        try {
            for (String s : sets) config.set(s);
            type = OpModeCompiler.compile(source, new File(out, "classes"));
        } catch (IllegalArgumentException | IllegalStateException e) {
            System.err.println(e.getMessage());
            System.exit(2);
            return;
        }

        double runSeconds = seconds != null ? seconds : type.getAnnotation(Autonomous.class) != null ? 30 : 10;
        String title = title(type);
        System.out.printf(Locale.US, "odyssey-sim: %s, INIT for %.1f s then %.1f s after START%n",
                title, config.number("sim.initSeconds"), runSeconds);

        Simulation sim = new Simulation(config);
        Thread watchdog = startWatchdog();
        sim.run(type, runSeconds);
        watchdog.interrupt();

        Report report = new Report(sim, title);
        report.printSummary(System.out);
        report.write(out);
        System.out.println("  report: " + new File(out, "report.html").getPath());
        System.exit(sim.failure() == null ? 0 : 1);
    }

    static String title(Class<?> type) {
        Autonomous auto = type.getAnnotation(Autonomous.class);
        TeleOp teleOp = type.getAnnotation(TeleOp.class);
        String display = auto != null ? auto.name() : teleOp != null ? teleOp.name() : "";
        return display.isEmpty() || display.equals(type.getSimpleName())
                ? type.getSimpleName() : type.getSimpleName() + " (\"" + display + "\")";
    }

    // An OpMode that busy-waits without hardware I/O never lets simulated time move.
    private static Thread startWatchdog() {
        Thread main = Thread.currentThread();
        Thread t = new Thread(() -> {
            long last = -1;
            int stuck = 0;
            try {
                while (true) {
                    Thread.sleep(1000);
                    long now = SimClock.nanoTime();
                    stuck = now == last ? stuck + 1 : 0;
                    last = now;
                    if (stuck >= 10) {
                        System.err.printf(Locale.US, "%nodyssey-sim: simulated time hasn't moved for 10 s (stuck at %.3f s). The OpMode is%n"
                                + "probably looping without hardware I/O, sleep(), idle() or opModeIsActive(), e.g. busy-waiting%n"
                                + "on an ElapsedTime. It's here:%n", now / 1e9);
                        for (StackTraceElement e : main.getStackTrace()) System.err.println("    at " + e);
                        System.exit(3);
                    }
                }
            } catch (InterruptedException e) {
                // run finished
            }
        }, "odyssey-sim-watchdog");
        t.setDaemon(true);
        t.start();
        return t;
    }

    private static String value(String[] args, int i, String flag) {
        if (i >= args.length) fail(flag + " needs a value");
        return args[i];
    }

    private static void fail(String message) {
        System.err.println("odyssey-sim: " + message);
        System.err.println(USAGE);
        System.exit(2);
    }
}
