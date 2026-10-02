package org.firstinspires.ftc.teamcode.odysseysim;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;

// What happened in one run: a console summary, report.html (path vs where the robot went, plus
// speed / wheel power / battery over time and the telemetry log) and trace.csv.
final class Report {
    static final double ARRIVAL_TOLERANCE_MM = 25.0;
    private static final String[] WHEELS = {"FL", "FR", "BL", "BR"};
    private static final String[] WHEEL_COLORS = {"#2563eb", "#dc2626", "#16a34a", "#9333ea"};

    private final Simulation sim;
    private final String title;
    private final List<Trace.Sample> samples;
    final List<PathProbe.SampledPath> paths = new ArrayList<>();   // field coordinates

    double distanceTravelled;
    double endError = Double.NaN;
    double endHeadingError = Double.NaN;
    double maxCrossTrack = Double.NaN;
    double arrivalTime = Double.NaN;
    double odometryError;
    double minBattery = Double.POSITIVE_INFINITY;
    double peakPower;
    double wallHitTime = Double.NaN;
    double loopMean = Double.NaN;
    double loopMax = Double.NaN;

    Report(Simulation sim, String title) {
        this.sim = sim;
        this.title = title;
        this.samples = sim.trace.samples;
        if (sim.opMode() != null) {
            double[] origin = sim.pinpointOrigin();
            try {
                for (PathProbe.SampledPath p : PathProbe.find(sim.opMode())) paths.add(toField(p, origin));
            } catch (RuntimeException e) {
                sim.warn("couldn't read the OpMode's path for the report: " + e);
            }
        }
        compute();
    }

    private static PathProbe.SampledPath toField(PathProbe.SampledPath p, double[] origin) {
        double c = Math.cos(origin[2]);
        double s = Math.sin(origin[2]);
        double[][] pts = new double[p.points.length][];
        for (int i = 0; i < pts.length; i++) {
            double[] q = p.points[i];
            pts[i] = new double[] {origin[0] + c * q[0] - s * q[1], origin[1] + s * q[0] + c * q[1], DrivePhysics.normalize(q[2] + origin[2])};
        }
        return new PathProbe.SampledPath(p.name, p.length, pts);
    }

    private double[] target() {
        if (paths.isEmpty()) return null;
        double[][] pts = paths.get(paths.size() - 1).points;
        return pts[pts.length - 1];
    }

    private void compute() {
        Trace.Sample last = samples.get(samples.size() - 1);
        double[] target = target();
        Trace.Sample prev = null;
        for (Trace.Sample s : samples) {
            if (s.time < 0) continue;
            if (prev != null) distanceTravelled += Math.hypot(s.trueX - prev.trueX, s.trueY - prev.trueY);
            prev = s;
            minBattery = Math.min(minBattery, s.battery);
            for (double p : s.power) peakPower = Math.max(peakPower, Math.abs(p));
            if (target != null) {
                double d = distanceToPaths(s.trueX, s.trueY);
                maxCrossTrack = Double.isNaN(maxCrossTrack) ? d : Math.max(maxCrossTrack, d);
                if (Double.isNaN(arrivalTime) && Math.hypot(s.trueX - target[0], s.trueY - target[1]) <= ARRIVAL_TOLERANCE_MM) {
                    arrivalTime = s.time;
                }
            }
        }
        if (minBattery == Double.POSITIVE_INFINITY) minBattery = last.battery;
        if (target != null) {
            endError = Math.hypot(last.trueX - target[0], last.trueY - target[1]);
            endHeadingError = Math.toDegrees(DrivePhysics.normalize(last.trueHeading - target[2]));
        }
        odometryError = Math.hypot(last.odoX - last.trueX, last.odoY - last.trueY);
        if (!Double.isNaN(sim.physics.firstWallHit)) {
            wallHitTime = sim.physics.firstWallHit - (sim.started() ? sim.startTime() : sim.initSeconds());
        }
        if (!sim.loopPeriodsMs.isEmpty()) {
            double sum = 0;
            for (double v : sim.loopPeriodsMs) sum += v;
            loopMean = sum / sim.loopPeriodsMs.size();
            loopMax = Collections.max(sim.loopPeriodsMs);
        }
    }

    private double distanceToPaths(double x, double y) {
        double best = Double.POSITIVE_INFINITY;
        for (PathProbe.SampledPath p : paths) {
            for (int i = 0; i + 1 < p.points.length; i++) {
                double[] a = p.points[i];
                double[] b = p.points[i + 1];
                double dx = b[0] - a[0];
                double dy = b[1] - a[1];
                double len2 = dx * dx + dy * dy;
                double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((x - a[0]) * dx + (y - a[1]) * dy) / len2));
                best = Math.min(best, Math.hypot(x - (a[0] + t * dx), y - (a[1] + t * dy)));
            }
        }
        return best;
    }

    List<String> notes() {
        List<String> notes = new ArrayList<>(sim.warnings());
        for (Map.Entry<String, Integer> e : sim.nanPowerCounts().entrySet()) {
            notes.add(String.format(Locale.US, "\"%s\" was sent NaN power %d times", e.getKey(), e.getValue()));
        }
        if (!Double.isNaN(wallHitTime)) notes.add(String.format(Locale.US, "the robot hit the field wall at %.2f s", wallHitTime));
        if (sim.started() && peakPower > 0 && distanceTravelled < 5.0) {
            notes.add("the robot barely moved even though the drive motors were powered");
        }
        if (sim.started() && peakPower == 0) notes.add("the drive motors were never powered after START");
        if (sim.opMode() != null && paths.isEmpty()) notes.add("no odyssey Path found in the OpMode, so there's nothing to compare against");
        return notes;
    }

    // ---- console ---------------------------------------------------------------------------------

    void printSummary(PrintStream out) {
        DrivePhysics p = sim.physics;
        out.println();
        out.println("odyssey-sim: " + title);
        if (sim.failure() != null) {
            out.println("  THE OPMODE THREW AN EXCEPTION (on the robot: \"User code threw an uncaught exception\"):");
            for (String line : stackTrace(sim.failure()).split("\\R")) out.println("    " + line);
        }
        out.printf(Locale.US, "  simulated robot: kS %.4f, kV %.6f, kA %.6f%n", p.equivalentKs(), p.equivalentKv(), p.equivalentKa());
        if (sim.started()) {
            out.printf(Locale.US, "  ran %.1f s after START", sim.stopTime() - sim.startTime());
            if (!Double.isNaN(loopMean)) out.printf(Locale.US, "; loop %.1f ms (%.0f Hz), max %.1f ms", loopMean, 1000 / loopMean, loopMax);
            out.println();
        } else {
            out.println("  never got past INIT");
        }
        for (PathProbe.SampledPath path : paths) {
            double[] end = path.points[path.points.length - 1];
            out.printf(Locale.US, "  path: %.1f mm, ends at (%.1f, %.1f) heading %.1f deg%n", path.length, end[0], end[1], Math.toDegrees(end[2]));
        }
        Trace.Sample last = samples.get(samples.size() - 1);
        out.printf(Locale.US, "  robot: ended at (%.1f, %.1f) heading %.1f deg after moving %.1f mm%n",
                last.trueX, last.trueY, Math.toDegrees(last.trueHeading), distanceTravelled);
        if (target() != null) {
            out.printf(Locale.US, "  vs path: ended %.1f mm from its end (heading %.1f deg off), strayed up to %.1f mm from it, %s%n",
                    endError, endHeadingError, maxCrossTrack, arrivalText());
        }
        out.printf(Locale.US, "  odometry vs truth at the end: %.1f mm; battery min %.2f V; peak wheel power %.2f%n",
                odometryError, minBattery, peakPower);
        for (String note : notes()) out.println("  ! " + note);
    }

    private String arrivalText() {
        return Double.isNaN(arrivalTime)
                ? String.format(Locale.US, "never got within %.0f mm of the end", ARRIVAL_TOLERANCE_MM)
                : String.format(Locale.US, "within %.0f mm of the end at %.2f s", ARRIVAL_TOLERANCE_MM, arrivalTime);
    }

    // ---- files ------------------------------------------------------------------------------------

    void write(File dir) throws IOException {
        if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("can't create " + dir);
        try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(new File(dir, "trace.csv").toPath(), StandardCharsets.UTF_8))) {
            w.println("t_s,phase,true_x_mm,true_y_mm,true_heading_deg,odo_x_mm,odo_y_mm,odo_heading_deg,"
                    + "speed_mm_s,omega_deg_s,power_fl,power_fr,power_bl,power_br,battery_v");
            for (Trace.Sample s : samples) {
                w.printf(Locale.US, "%.4f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f%n",
                        s.time, s.phase, s.trueX, s.trueY, Math.toDegrees(s.trueHeading), s.odoX, s.odoY,
                        Math.toDegrees(s.odoHeading), s.speed, Math.toDegrees(s.omega),
                        s.power[0], s.power[1], s.power[2], s.power[3], s.battery);
            }
        }
        try (Writer w = Files.newBufferedWriter(new File(dir, "report.html").toPath(), StandardCharsets.UTF_8)) {
            w.write(html());
        }
    }

    private String html() {
        StringBuilder h = new StringBuilder();
        h.append("<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\">\n")
                .append("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n")
                .append("<title>odyssey-sim: ").append(esc(title)).append("</title>\n<style>\n")
                .append("body{font:14px/1.45 system-ui,-apple-system,Segoe UI,sans-serif;margin:24px auto;max-width:880px;padding:0 16px;color:#1f2328;background:#fff}\n")
                .append("h1{font-size:22px;margin:0 0 4px}h2{font-size:16px;margin:28px 0 8px}.sub{color:#59636e;margin:0 0 16px}\n")
                .append("table{border-collapse:collapse;width:100%}td{padding:5px 8px;border-bottom:1px solid #e5e7eb;vertical-align:top}\n")
                .append("td:first-child{color:#59636e;width:32%}\n")
                .append(".note{background:#fff8c5;border:1px solid #d4a72c;border-radius:6px;padding:8px 12px;margin:10px 0}\n")
                .append(".err{background:#ffebe9;border:1px solid #cf222e;border-radius:6px;padding:8px 12px;margin:10px 0;white-space:pre-wrap;font:12px ui-monospace,Consolas,monospace}\n")
                .append("pre{background:#f6f8fa;padding:10px 12px;border-radius:6px;overflow:auto;font:12px/1.4 ui-monospace,Consolas,monospace}\n")
                .append("svg{width:100%;height:auto;display:block}text{font:11px system-ui,sans-serif;fill:#59636e}\n")
                .append("</style></head><body>\n");
        h.append("<h1>").append(esc(title)).append("</h1>\n")
                .append("<p class=\"sub\">odyssey-sim. Times are seconds after START; field coordinates in mm with the origin at the field center.</p>\n");
        if (sim.failure() != null) {
            h.append("<div class=\"err\"><b>The OpMode threw an exception</b>\n").append(esc(stackTrace(sim.failure()))).append("</div>\n");
        }
        for (String note : notes()) h.append("<div class=\"note\">").append(esc(note)).append("</div>\n");

        DrivePhysics p = sim.physics;
        Trace.Sample last = samples.get(samples.size() - 1);
        h.append("<table>\n");
        row(h, "Simulated robot", String.format(Locale.US, "kS %.4f, kV %.6f, kA %.6f", p.equivalentKs(), p.equivalentKv(), p.equivalentKa()));
        if (sim.started()) row(h, "Ran", String.format(Locale.US, "%.1f s after START", sim.stopTime() - sim.startTime()));
        if (!Double.isNaN(loopMean)) row(h, "Loop", String.format(Locale.US, "%.1f ms (%.0f Hz), max %.1f ms", loopMean, 1000 / loopMean, loopMax));
        for (PathProbe.SampledPath path : paths) {
            double[] end = path.points[path.points.length - 1];
            row(h, "Path", String.format(Locale.US, "%.1f mm, ends at (%.1f, %.1f) heading %.1f&deg;", path.length, end[0], end[1], Math.toDegrees(end[2])));
        }
        row(h, "Robot ended at", String.format(Locale.US, "(%.1f, %.1f) heading %.1f&deg; after moving %.1f mm",
                last.trueX, last.trueY, Math.toDegrees(last.trueHeading), distanceTravelled));
        if (target() != null) {
            row(h, "Distance from path end", String.format(Locale.US, "%.1f mm, heading %.1f&deg; off", endError, endHeadingError));
            row(h, "Furthest from the path", String.format(Locale.US, "%.1f mm", maxCrossTrack));
            row(h, "Reached the end", esc(arrivalText()));
        }
        row(h, "Odometry vs truth at end", String.format(Locale.US, "%.1f mm", odometryError));
        row(h, "Battery / power", String.format(Locale.US, "min %.2f V; peak wheel power %.2f", minBattery, peakPower));
        h.append("</table>\n");

        h.append("<h2>Path vs where the robot went</h2>\n").append(trajectorySvg());
        h.append("<h2>Over time</h2>\n").append(timeSeriesSvg());
        h.append("<h2>Telemetry log</h2>\n<pre>");
        if (sim.telemetry.logEntries.isEmpty()) h.append("(nothing logged)");
        for (SimTelemetry.Entry e : sim.telemetry.logEntries) {
            h.append(esc(String.format(Locale.US, "[%+8.3f s] %s", e.time, e.text))).append('\n');
        }
        h.append("</pre>\n<h2>Last telemetry</h2>\n<pre>").append(esc(sim.telemetry.lastFrame())).append("</pre>\n</body></html>\n");
        return h.toString();
    }

    private static void row(StringBuilder h, String label, String valueHtml) {
        h.append("<tr><td>").append(esc(label)).append("</td><td>").append(valueHtml).append("</td></tr>\n");
    }

    private String trajectorySvg() {
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (Trace.Sample s : samples) {
            minX = Math.min(minX, Math.min(s.trueX, s.odoX));
            maxX = Math.max(maxX, Math.max(s.trueX, s.odoX));
            minY = Math.min(minY, Math.min(s.trueY, s.odoY));
            maxY = Math.max(maxY, Math.max(s.trueY, s.odoY));
        }
        for (PathProbe.SampledPath p : paths) {
            for (double[] q : p.points) {
                minX = Math.min(minX, q[0]);
                maxX = Math.max(maxX, q[0]);
                minY = Math.min(minY, q[1]);
                maxY = Math.max(maxY, q[1]);
            }
        }
        double robot = Math.hypot(sim.physics.halfLength, sim.physics.halfWidth) * 1000.0;
        double margin = Math.max(robot, 0.08 * Math.max(maxX - minX, maxY - minY));
        minX -= margin;
        maxX += margin;
        minY -= margin;
        maxY += margin;
        final double width = 840;
        double scale = Math.min(width / (maxX - minX), 620 / (maxY - minY));
        double extraX = (width / scale - (maxX - minX)) / 2;   // center the plot horizontally
        minX -= extraX;
        maxX += extraX;
        final double height = (maxY - minY) * scale;
        final double x0 = minX, y1 = maxY, sc = scale;

        StringBuilder g = new StringBuilder();
        g.append(String.format(Locale.US, "<svg viewBox=\"0 0 %.0f %.0f\" role=\"img\" aria-label=\"path and robot trajectory\">\n", width, height + 24));
        g.append(String.format(Locale.US, "<rect width=\"%.0f\" height=\"%.0f\" fill=\"#fbfbfc\" stroke=\"#d0d7de\"/>\n", width, height));
        double step = niceStep(Math.max(maxX - minX, maxY - minY) / 8);
        for (double gx = Math.ceil(minX / step) * step; gx <= maxX; gx += step) {
            double px = (gx - x0) * sc;
            g.append(String.format(Locale.US, "<line x1=\"%.1f\" y1=\"0\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"#eaeef2\"/>", px, px, height));
            g.append(String.format(Locale.US, "<text x=\"%.1f\" y=\"%.1f\" text-anchor=\"middle\">%s</text>\n", px, height + 14, fmt(gx)));
        }
        for (double gy = Math.ceil(minY / step) * step; gy <= maxY; gy += step) {
            double py = (y1 - gy) * sc;
            g.append(String.format(Locale.US, "<line x1=\"0\" y1=\"%.1f\" x2=\"%.0f\" y2=\"%.1f\" stroke=\"#eaeef2\"/>", py, width, py));
            g.append(String.format(Locale.US, "<text x=\"4\" y=\"%.1f\">%s</text>\n", py - 3, fmt(gy)));
        }
        double half = sim.physics.fieldHalf * 1000.0;
        g.append(String.format(Locale.US, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"none\" stroke=\"#8c959f\" stroke-width=\"3\"/>\n",
                (-half - x0) * sc, (y1 - half) * sc, 2 * half * sc, 2 * half * sc));

        Trace.Sample first = samples.get(0);
        Trace.Sample last = samples.get(samples.size() - 1);
        g.append(robotOutline(first.trueX, first.trueY, first.trueHeading, "#8c959f", x0, y1, sc));
        for (PathProbe.SampledPath p : paths) {
            g.append("<polyline fill=\"none\" stroke=\"#2563eb\" stroke-width=\"4\" stroke-opacity=\"0.55\" stroke-linejoin=\"round\" points=\"");
            for (double[] q : p.points) g.append(String.format(Locale.US, "%.1f,%.1f ", (q[0] - x0) * sc, (y1 - q[1]) * sc));
            g.append("\"/>\n");
            double[] end = p.points[p.points.length - 1];
            g.append(String.format(Locale.US, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"5\" fill=\"#2563eb\"/>\n", (end[0] - x0) * sc, (y1 - end[1]) * sc));
        }
        g.append(trajectory(true, "#16a34a", "1.5", "5 4", x0, y1, sc));
        g.append(trajectory(false, "#ea580c", "2", null, x0, y1, sc));
        g.append(robotOutline(last.trueX, last.trueY, last.trueHeading, "#ea580c", x0, y1, sc));
        double ly = height - 52;
        g.append(legend(10, ly, "#2563eb", "path (from the OpMode)"));
        g.append(legend(10, ly + 16, "#ea580c", "robot, true position"));
        g.append(legend(10, ly + 32, "#16a34a", "robot, Pinpoint estimate"));
        return g.append("</svg>\n").toString();
    }

    private String trajectory(boolean odometry, String color, String strokeWidth, String dash, double x0, double y1, double sc) {
        StringBuilder g = new StringBuilder("<polyline fill=\"none\" stroke=\"").append(color)
                .append("\" stroke-width=\"").append(strokeWidth).append("\" stroke-linejoin=\"round\"");
        if (dash != null) g.append(" stroke-dasharray=\"").append(dash).append('"');
        g.append(" points=\"");
        for (Trace.Sample s : samples) {
            double x = odometry ? s.odoX : s.trueX;
            double y = odometry ? s.odoY : s.trueY;
            g.append(String.format(Locale.US, "%.1f,%.1f ", (x - x0) * sc, (y1 - y) * sc));
        }
        return g.append("\"/>\n").toString();
    }

    private String robotOutline(double x, double y, double heading, String color, double x0, double y1, double sc) {
        double hl = sim.physics.halfLength * 1000.0;
        double hw = sim.physics.halfWidth * 1000.0;
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double[][] corners = {{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}};
        StringBuilder g = new StringBuilder("<polygon fill=\"none\" stroke=\"").append(color).append("\" stroke-width=\"1.5\" points=\"");
        for (double[] k : corners) {
            g.append(String.format(Locale.US, "%.1f,%.1f ", (x + c * k[0] - s * k[1] - x0) * sc, (y1 - (y + s * k[0] + c * k[1])) * sc));
        }
        g.append("\"/>");
        g.append(String.format(Locale.US, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"1.5\"/>\n",
                (x - x0) * sc, (y1 - y) * sc, (x + c * hl - x0) * sc, (y1 - (y + s * hl)) * sc, color));
        return g.toString();
    }

    private static String legend(double x, double y, String color, String label) {
        return String.format(Locale.US, "<line x1=\"%.0f\" y1=\"%.0f\" x2=\"%.0f\" y2=\"%.0f\" stroke=\"%s\" stroke-width=\"3\"/>"
                + "<text x=\"%.0f\" y=\"%.0f\">%s</text>\n", x, y, x + 18, y, color, x + 24, y + 4, esc(label));
    }

    private String timeSeriesSvg() {
        // Show START until a second after the robot last moved.
        double tEnd = 1.0;
        for (Trace.Sample s : samples) {
            if (s.time >= 0 && (s.speed > 5 || Math.abs(s.omega) > 0.05)) tEnd = Math.max(tEnd, s.time + 1.0);
        }
        tEnd = Math.min(tEnd, samples.get(samples.size() - 1).time);
        List<Trace.Sample> window = new ArrayList<>();
        for (Trace.Sample s : samples) if (s.time >= 0 && s.time <= tEnd) window.add(s);
        if (window.size() < 2 || tEnd <= 0) return "<p>(nothing after START)</p>\n";

        final double width = 840, panel = 130, gap = 34, left = 52, plotWidth = width - left - 8;
        String[] titles = {"robot speed (mm/s)", "wheel power", "battery (V)"};
        StringBuilder g = new StringBuilder();
        g.append(String.format(Locale.US, "<svg viewBox=\"0 0 %.0f %.0f\" role=\"img\" aria-label=\"speed, wheel power and battery over time\">\n",
                width, 3 * (panel + gap)));
        for (int k = 0; k < 3; k++) {
            double top = k * (panel + gap) + 16;
            double lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
            for (Trace.Sample s : window) {
                for (double v : values(s, k)) {
                    lo = Math.min(lo, v);
                    hi = Math.max(hi, v);
                }
            }
            if (k == 0) lo = 0;
            if (k == 1) {
                lo = Math.min(lo, -1);
                hi = Math.max(hi, 1);
            }
            if (hi - lo < 1e-6) hi = lo + 1;
            double pad = (hi - lo) * 0.08;
            if (k != 0) lo -= pad;
            hi += pad;
            g.append(String.format(Locale.US, "<text x=\"%.0f\" y=\"%.0f\" style=\"fill:#1f2328\">%s</text>\n", left, top - 4, titles[k]));
            g.append(String.format(Locale.US, "<rect x=\"%.0f\" y=\"%.0f\" width=\"%.0f\" height=\"%.0f\" fill=\"#fbfbfc\" stroke=\"#d0d7de\"/>\n",
                    left, top, plotWidth, panel));
            double ystep = niceStep((hi - lo) / 4);
            for (double v = Math.ceil(lo / ystep) * ystep; v <= hi; v += ystep) {
                double y = top + panel - (v - lo) / (hi - lo) * panel;
                g.append(String.format(Locale.US, "<line x1=\"%.0f\" y1=\"%.1f\" x2=\"%.0f\" y2=\"%.1f\" stroke=\"#eaeef2\"/>"
                        + "<text x=\"%.0f\" y=\"%.1f\" text-anchor=\"end\">%s</text>\n", left, y, width - 8, y, left - 4, y + 4, fmt(v)));
            }
            double tstep = niceStep(tEnd / 8);
            for (double t = 0; t <= tEnd + 1e-9; t += tstep) {
                double x = left + t / tEnd * plotWidth;
                g.append(String.format(Locale.US, "<line x1=\"%.1f\" y1=\"%.0f\" x2=\"%.1f\" y2=\"%.0f\" stroke=\"#eaeef2\"/>", x, top, x, top + panel));
                if (k == 2) g.append(String.format(Locale.US, "<text x=\"%.1f\" y=\"%.0f\" text-anchor=\"middle\">%s s</text>", x, top + panel + 14, fmt(t)));
            }
            int series = k == 1 ? 4 : 1;
            for (int i = 0; i < series; i++) {
                String color = k == 1 ? WHEEL_COLORS[i] : (k == 0 ? "#ea580c" : "#59636e");
                g.append("<polyline fill=\"none\" stroke=\"").append(color).append("\" stroke-width=\"1.5\" points=\"");
                for (Trace.Sample s : window) {
                    double x = left + s.time / tEnd * plotWidth;
                    double y = top + panel - (values(s, k)[i] - lo) / (hi - lo) * panel;
                    g.append(String.format(Locale.US, "%.1f,%.1f ", x, y));
                }
                g.append("\"/>\n");
            }
            if (k == 1) {
                for (int w = 0; w < 4; w++) g.append(legend(width - 230 + w * 55, top - 8, WHEEL_COLORS[w], WHEELS[w]));
            }
        }
        return g.append("</svg>\n").toString();
    }

    private static double[] values(Trace.Sample s, int panel) {
        if (panel == 0) return new double[] {s.speed};
        if (panel == 1) return s.power;
        return new double[] {s.battery};
    }

    static double niceStep(double raw) {
        if (!(raw > 0)) return 1;
        double mag = Math.pow(10, Math.floor(Math.log10(raw)));
        double n = raw / mag;
        return (n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10) * mag;
    }

    private static String fmt(double v) {
        if (Math.abs(v) < 1e-9) return "0";
        if (Math.abs(v - Math.rint(v)) < 1e-9) return String.format(Locale.US, "%.0f", v);
        return String.format(Locale.US, "%.2f", v).replaceAll("0+$", "").replaceAll("\\.$", "");
    }

    private static String stackTrace(Throwable t) {
        StringWriter sw = new StringWriter();
        t.printStackTrace(new PrintWriter(sw));
        return sw.toString();
    }

    private static String esc(String s) {
        return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("\"", "&quot;");
    }
}
