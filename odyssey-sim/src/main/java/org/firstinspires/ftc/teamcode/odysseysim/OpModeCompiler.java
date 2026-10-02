package org.firstinspires.ftc.teamcode.odysseysim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Modifier;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;

import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;

// Compiles the OpMode .java file you pass against the simulator's classpath (the FTC stand-ins,
// odyssey-core, TeamCode's odyssey classes) and loads the OpMode in it.
final class OpModeCompiler {
    private OpModeCompiler() {}

    static Class<? extends OpMode> compile(File source, File classesDir) throws IOException {
        if (!source.isFile()) throw new IllegalArgumentException("odyssey-sim: no such file: " + source.getPath());
        JavaCompiler javac = ToolProvider.getSystemJavaCompiler();
        if (javac == null) {
            throw new IllegalStateException("odyssey-sim: compiling the OpMode needs a JDK, but this Java ("
                    + System.getProperty("java.home") + ") has no compiler");
        }
        deleteRecursively(classesDir);
        if (!classesDir.mkdirs()) throw new IOException("can't create " + classesDir);

        DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<>();
        boolean ok;
        try (StandardJavaFileManager files = javac.getStandardFileManager(diagnostics, Locale.US, StandardCharsets.UTF_8)) {
            List<String> options = Arrays.asList("-d", classesDir.getPath(), "-classpath", System.getProperty("java.class.path"),
                    "-proc:none", "-nowarn", "-encoding", "UTF-8");
            ok = javac.getTask(null, files, diagnostics, options, null, files.getJavaFileObjects(source)).call();
        }
        if (!ok) throw new IllegalArgumentException(describeErrors(source, diagnostics));

        ClassLoader loader = new URLClassLoader(new URL[] {classesDir.toURI().toURL()}, OpModeCompiler.class.getClassLoader());
        List<Class<? extends OpMode>> opModes = new ArrayList<>();
        Path root = classesDir.toPath();
        try (Stream<Path> classFiles = Files.walk(root)) {
            for (Path p : (Iterable<Path>) classFiles.filter(f -> f.toString().endsWith(".class"))::iterator) {
                String name = root.relativize(p).toString().replace(File.separatorChar, '.');
                name = name.substring(0, name.length() - ".class".length());
                try {
                    Class<?> c = Class.forName(name, false, loader);
                    int mods = c.getModifiers();
                    if (OpMode.class.isAssignableFrom(c) && !Modifier.isAbstract(mods) && Modifier.isPublic(mods)
                            && (c.getEnclosingClass() == null || Modifier.isStatic(mods))) {
                        opModes.add(c.asSubclass(OpMode.class));
                    }
                } catch (ClassNotFoundException | LinkageError e) {
                    // not loadable on its own; not an OpMode we can run
                }
            }
        }

        String fileName = source.getName().replaceAll("\\.java$", "");
        if (opModes.size() == 1) return opModes.get(0);
        for (Class<? extends OpMode> c : opModes) {
            if (c.getSimpleName().equals(fileName)) return c;
        }
        throw new IllegalArgumentException(opModes.isEmpty()
                ? "odyssey-sim: " + source.getName() + " has no OpMode (a public class extending OpMode or LinearOpMode)"
                : "odyssey-sim: " + source.getName() + " has several OpModes and none is named " + fileName);
    }

    private static String describeErrors(File source, DiagnosticCollector<JavaFileObject> diagnostics) {
        StringBuilder msg = new StringBuilder("odyssey-sim: " + source.getName() + " doesn't compile:");
        boolean missing = false;
        for (Diagnostic<? extends JavaFileObject> d : diagnostics.getDiagnostics()) {
            if (d.getKind() != Diagnostic.Kind.ERROR) continue;
            String text = d.getMessage(Locale.US);
            msg.append(String.format(Locale.US, "%n  line %d: %s", d.getLineNumber(), text.replace("\n", "\n    ")));
            if (text.contains("cannot find symbol") || text.contains("does not exist")) missing = true;
        }
        if (missing) {
            msg.append(String.format("%nThe simulator has the FTC SDK classes OpModes usually use, odyssey-core and TeamCode's"
                    + "%nodyssey package. A class outside those isn't available (see odyssey-sim/README.md)."));
        }
        return msg.toString();
    }

    private static void deleteRecursively(File dir) throws IOException {
        if (!dir.exists()) return;
        try (Stream<Path> paths = Files.walk(dir.toPath())) {
            for (Path p : (Iterable<Path>) paths.sorted(Comparator.reverseOrder())::iterator) Files.delete(p);
        }
    }
}
