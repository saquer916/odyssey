package org.firstinspires.ftc.robotcore.external;

// odyssey-sim stand-in.
public interface Telemetry {

    interface Item {
        String getCaption();
        Item setCaption(String caption);
        Item setValue(String format, Object... args);
        Item setValue(Object value);
        Item setRetained(Boolean retained);
        boolean isRetained();
        Item addData(String caption, String format, Object... args);
        Item addData(String caption, Object value);
    }

    interface Line {
        Item addData(String caption, String format, Object... args);
        Item addData(String caption, Object value);
    }

    interface Log {
        void add(String entry);
        void add(String format, Object... args);
        void clear();
        int getCapacity();
        void setCapacity(int capacity);
    }

    Item addData(String caption, String format, Object... args);
    Item addData(String caption, Object value);
    boolean removeItem(Item item);
    Line addLine();
    Line addLine(String lineCaption);
    boolean removeLine(Line line);
    void clear();
    void clearAll();
    boolean update();
    Log log();
    boolean isAutoClear();
    void setAutoClear(boolean autoClear);
    int getMsTransmissionInterval();
    void setMsTransmissionInterval(int msTransmissionInterval);
    String getItemSeparator();
    void setItemSeparator(String itemSeparator);
    String getCaptionValueSeparator();
    void setCaptionValueSeparator(String captionValueSeparator);
    void speak(String text);
}
