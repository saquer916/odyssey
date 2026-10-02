package org.firstinspires.ftc.teamcode.odysseysim;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

// Telemetry as the Driver Station sees it: update() sends at most once per transmission interval
// (250 ms by default) and auto-clears; log() lines are kept and echoed to the console.
final class SimTelemetry implements Telemetry {

    static final class Entry {
        final double time;
        final String text;

        Entry(double time, String text) {
            this.time = time;
            this.text = text;
        }
    }

    private final Simulation sim;
    private final List<Object> items = new ArrayList<>();
    private final LogImpl log = new LogImpl();
    final List<Entry> logEntries = new ArrayList<>();
    private boolean autoClear = true;
    private boolean dirty;
    private int msTransmissionInterval = 250;
    private double lastTransmit = Double.NEGATIVE_INFINITY;
    private String itemSeparator = " | ";
    private String captionValueSeparator = " : ";
    private String lastFrame = "";

    SimTelemetry(Simulation sim) {
        this.sim = sim;
    }

    String lastFrame() {
        return lastFrame;
    }

    // The SDK sends telemetry after each init()/init_loop()/loop() of an iterative OpMode.
    void autoUpdate() {
        if (dirty) update();
    }

    private final class ItemImpl implements Item {
        private String caption;
        private String value;
        private boolean retained;

        ItemImpl(String caption, String value) {
            this.caption = caption;
            this.value = value;
        }

        @Override public String getCaption() { return caption; }
        @Override public Item setCaption(String caption) { this.caption = caption; dirty = true; return this; }
        @Override public Item setValue(String format, Object... args) { value = String.format(format, args); dirty = true; return this; }
        @Override public Item setValue(Object value) { this.value = String.valueOf(value); dirty = true; return this; }
        @Override public Item setRetained(Boolean retained) { this.retained = retained != null && retained; return this; }
        @Override public boolean isRetained() { return retained; }
        @Override public Item addData(String caption, String format, Object... args) { return SimTelemetry.this.addData(caption, format, args); }
        @Override public Item addData(String caption, Object value) { return SimTelemetry.this.addData(caption, value); }

        String render() {
            return caption + captionValueSeparator + value;
        }
    }

    private final class LineImpl implements Line {
        private final String caption;
        private final List<ItemImpl> lineItems = new ArrayList<>();

        LineImpl(String caption) {
            this.caption = caption;
        }

        @Override public Item addData(String caption, String format, Object... args) { return add(new ItemImpl(caption, String.format(format, args))); }
        @Override public Item addData(String caption, Object value) { return add(new ItemImpl(caption, String.valueOf(value))); }

        private Item add(ItemImpl item) {
            lineItems.add(item);
            dirty = true;
            return item;
        }

        String render() {
            StringBuilder sb = new StringBuilder(caption);
            for (int i = 0; i < lineItems.size(); i++) {
                if (i > 0) sb.append(itemSeparator);
                else if (caption.length() > 0) sb.append(' ');
                sb.append(lineItems.get(i).render());
            }
            return sb.toString();
        }
    }

    private final class LogImpl implements Log {
        private int capacity = 9;

        @Override
        public void add(String entry) {
            Entry e = new Entry(sim.timeSinceStart(), entry);
            logEntries.add(e);
            System.out.printf("  [%+8.3f s] %s%n", e.time, entry);
        }

        @Override public void add(String format, Object... args) { add(String.format(format, args)); }
        @Override public void clear() { logEntries.clear(); }
        @Override public int getCapacity() { return capacity; }
        @Override public void setCapacity(int capacity) { this.capacity = capacity; }
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return addItem(new ItemImpl(caption, String.format(format, args)));
    }

    @Override
    public Item addData(String caption, Object value) {
        return addItem(new ItemImpl(caption, String.valueOf(value)));
    }

    private Item addItem(ItemImpl item) {
        items.add(item);
        dirty = true;
        return item;
    }

    @Override public boolean removeItem(Item item) { return items.remove(item); }
    @Override public Line addLine() { return addLine(""); }

    @Override
    public Line addLine(String lineCaption) {
        LineImpl line = new LineImpl(lineCaption);
        items.add(line);
        dirty = true;
        return line;
    }

    @Override public boolean removeLine(Line line) { return items.remove(line); }

    @Override
    public void clear() {
        Iterator<Object> it = items.iterator();
        while (it.hasNext()) {
            Object o = it.next();
            if (!(o instanceof ItemImpl) || !((ItemImpl) o).isRetained()) it.remove();
        }
    }

    @Override
    public void clearAll() {
        items.clear();
        log.clear();
    }

    @Override
    public boolean update() {
        StringBuilder sb = new StringBuilder();
        for (Object o : items) {
            if (sb.length() > 0) sb.append('\n');
            sb.append(o instanceof ItemImpl ? ((ItemImpl) o).render() : ((LineImpl) o).render());
        }
        lastFrame = sb.toString();
        boolean sent = sim.time() - lastTransmit >= msTransmissionInterval / 1000.0 - 1e-9;
        if (sent) lastTransmit = sim.time();
        if (autoClear) clear();
        dirty = false;
        return sent;
    }

    @Override public Log log() { return log; }
    @Override public boolean isAutoClear() { return autoClear; }
    @Override public void setAutoClear(boolean autoClear) { this.autoClear = autoClear; }
    @Override public int getMsTransmissionInterval() { return msTransmissionInterval; }
    @Override public void setMsTransmissionInterval(int ms) { msTransmissionInterval = ms; }
    @Override public String getItemSeparator() { return itemSeparator; }
    @Override public void setItemSeparator(String itemSeparator) { this.itemSeparator = itemSeparator; }
    @Override public String getCaptionValueSeparator() { return captionValueSeparator; }
    @Override public void setCaptionValueSeparator(String separator) { captionValueSeparator = separator; }
    @Override public void speak(String text) {}
}
