package org.firstinspires.ftc.teamcode.odysseysim;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.Properties;
import java.util.Set;
import java.util.TreeSet;

// Robot and simulator settings. Defaults are in src/main/resources/odysseysim/robot.properties;
// --set key=value overrides one, and only keys that exist there (so a typo is an error).
public final class SimConfig {
    private static final String DEFAULTS = "/odysseysim/robot.properties";

    private final Properties props = new Properties();
    private final Set<String> knownKeys;

    private SimConfig() {
        try (InputStream in = SimConfig.class.getResourceAsStream(DEFAULTS)) {
            if (in == null) throw new IllegalStateException("odyssey-sim: " + DEFAULTS + " is missing from the classpath");
            props.load(new InputStreamReader(in, StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new IllegalStateException("odyssey-sim: can't read " + DEFAULTS, e);
        }
        knownKeys = new TreeSet<>(props.stringPropertyNames());
    }

    public static SimConfig defaults() {
        return new SimConfig();
    }

    public SimConfig set(String keyEqualsValue) {
        int eq = keyEqualsValue.indexOf('=');
        if (eq <= 0) throw new IllegalArgumentException("odyssey-sim: expected key=value, got \"" + keyEqualsValue + "\"");
        String key = keyEqualsValue.substring(0, eq).trim();
        if (!knownKeys.contains(key)) {
            throw new IllegalArgumentException("odyssey-sim: unknown setting \"" + key + "\"; see robot.properties for the list");
        }
        props.setProperty(key, keyEqualsValue.substring(eq + 1).trim());
        return this;
    }

    public String string(String key) {
        String value = props.getProperty(key);
        if (value == null) throw new IllegalArgumentException("odyssey-sim: no setting \"" + key + "\"");
        return value.trim();
    }

    public double number(String key) {
        String value = string(key);
        try {
            return Double.parseDouble(value);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("odyssey-sim: setting \"" + key + "\" should be a number, got \"" + value + "\"");
        }
    }

    public boolean flag(String key) {
        return Boolean.parseBoolean(string(key));
    }
}
