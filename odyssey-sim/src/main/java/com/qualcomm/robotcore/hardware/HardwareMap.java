package com.qualcomm.robotcore.hardware;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Function;

// odyssey-sim stand-in. Devices come from the simulated robot; asking for a name or type that isn't
// on it throws, like the SDK does with a missing device in the robot configuration.
public class HardwareMap implements Iterable<HardwareDevice> {

    public class DeviceMapping<D extends HardwareDevice> implements Iterable<D> {
        private final Class<D> deviceTypeClass;
        private final Map<String, D> map = new LinkedHashMap<>();

        DeviceMapping(Class<D> deviceTypeClass) {
            this.deviceTypeClass = deviceTypeClass;
        }

        public Class<D> getDeviceTypeClass() { return deviceTypeClass; }

        public D get(String deviceName) {
            D device = map.get(deviceName);
            if (device == null) throw notFound(deviceName, deviceTypeClass);
            return device;
        }

        public void put(String deviceName, D device) { map.put(deviceName, device); }
        public boolean contains(String deviceName) { return map.containsKey(deviceName); }
        public Set<Map.Entry<String, D>> entrySet() { return map.entrySet(); }
        public int size() { return map.size(); }
        @Override public Iterator<D> iterator() { return map.values().iterator(); }
    }

    public final DeviceMapping<DcMotor> dcMotor = new DeviceMapping<>(DcMotor.class);
    public final DeviceMapping<VoltageSensor> voltageSensor = new DeviceMapping<>(VoltageSensor.class);

    private final Map<String, List<Object>> devices = new LinkedHashMap<>();
    // Devices whose class depends on what the OpMode asks for (TeamCode's or the SDK's Pinpoint driver).
    private final Map<String, Function<Class<?>, Object>> factories = new HashMap<>();

    public void put(String deviceName, HardwareDevice device) {
        devices.computeIfAbsent(deviceName, k -> new ArrayList<>()).add(device);
        if (device instanceof DcMotor) dcMotor.put(deviceName, (DcMotor) device);
        if (device instanceof VoltageSensor) voltageSensor.put(deviceName, (VoltageSensor) device);
    }

    // Not in the SDK: a device created on first request, as the class that was requested.
    public void putFactory(String deviceName, Function<Class<?>, Object> factory) {
        factories.put(deviceName, factory);
    }

    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) throw notFound(deviceName, classOrInterface);
        return result;
    }

    public HardwareDevice get(String deviceName) {
        List<Object> list = devices.get(deviceName);
        if (list != null) {
            for (Object o : list) if (o instanceof HardwareDevice) return (HardwareDevice) o;
        }
        throw new IllegalArgumentException(String.format(
                "Unable to find a hardware device with the name \"%s\" (the simulated robot has %s)", deviceName, knownNames()));
    }

    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        List<Object> list = devices.get(deviceName);
        if (list != null) {
            for (Object o : list) if (classOrInterface.isInstance(o)) return classOrInterface.cast(o);
        }
        Function<Class<?>, Object> factory = factories.get(deviceName);
        if (factory != null) {
            Object created = factory.apply(classOrInterface);
            if (created != null && classOrInterface.isInstance(created)) {
                devices.computeIfAbsent(deviceName, k -> new ArrayList<>()).add(created);
                return classOrInterface.cast(created);
            }
        }
        return null;
    }

    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new ArrayList<>();
        for (List<Object> list : devices.values()) {
            for (Object o : list) if (classOrInterface.isInstance(o)) result.add(classOrInterface.cast(o));
        }
        return result;
    }

    public int size() {
        return devices.size() + factories.size();
    }

    @Override
    public Iterator<HardwareDevice> iterator() {
        List<HardwareDevice> all = new ArrayList<>();
        for (List<Object> list : devices.values()) {
            for (Object o : list) if (o instanceof HardwareDevice) all.add((HardwareDevice) o);
        }
        return all.iterator();
    }

    private Set<String> knownNames() {
        Set<String> known = new TreeSet<>(devices.keySet());
        known.addAll(factories.keySet());
        return known;
    }

    private IllegalArgumentException notFound(String deviceName, Class<?> type) {
        return new IllegalArgumentException(String.format(
                "Unable to find a hardware device with name \"%s\" and type %s (the simulated robot has %s)",
                deviceName, type.getSimpleName(), knownNames()));
    }
}
