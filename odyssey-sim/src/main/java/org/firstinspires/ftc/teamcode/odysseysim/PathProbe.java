package org.firstinspires.ftc.teamcode.odysseysim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.path.Path;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

// Finds the odyssey Paths an OpMode built (its own fields, plus one level into its objects, which
// catches Follower.path) so the report can draw them and measure the robot against them.
final class PathProbe {

    static final class SampledPath {
        final String name;
        final double length;
        final double[][] points;   // x mm, y mm, heading rad

        SampledPath(String name, double length, double[][] points) {
            this.name = name;
            this.length = length;
            this.points = points;
        }
    }

    private PathProbe() {}

    static List<SampledPath> find(OpMode opMode) {
        Map<Path, String> found = new IdentityHashMap<>();
        List<Path> order = new ArrayList<>();
        for (Class<?> c = opMode.getClass(); c != null && c != OpMode.class; c = c.getSuperclass()) {
            for (Field f : c.getDeclaredFields()) {
                if (Modifier.isStatic(f.getModifiers())) continue;
                Object value = read(f, opMode);
                collect(value, f.getName(), found, order);
                if (value != null && value.getClass().getName().startsWith("org.firstinspires.ftc.teamcode")) {
                    for (Field inner : value.getClass().getDeclaredFields()) {
                        if (!Modifier.isStatic(inner.getModifiers())) {
                            collect(read(inner, value), f.getName() + "." + inner.getName(), found, order);
                        }
                    }
                }
            }
        }
        List<SampledPath> sampled = new ArrayList<>();
        for (Path path : order) sampled.add(sample(found.get(path), path));
        return sampled;
    }

    private static void collect(Object value, String name, Map<Path, String> found, List<Path> order) {
        if (value instanceof Path && !found.containsKey(value)) {
            found.put((Path) value, name);
            order.add((Path) value);
        }
    }

    private static Object read(Field f, Object owner) {
        try {
            f.setAccessible(true);
            return f.get(owner);
        } catch (ReflectiveOperationException | RuntimeException e) {
            return null;
        }
    }

    // In the Pinpoint's frame, i.e. the OpMode's own coordinates.
    private static SampledPath sample(String name, Path path) {
        double length = path.getTotalLength();
        int n = (int) Math.max(2, Math.min(400, length / 2 + 2));
        double[][] points = new double[n][];
        for (int i = 0; i < n; i++) {
            Pose2d p = path.getPointOnPath(length * i / (n - 1));
            points[i] = new double[] {p.getX(), p.getY(), p.getHeading()};
        }
        return new SampledPath(name, length, points);
    }
}
