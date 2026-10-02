package com.qualcomm.robotcore.hardware;

// odyssey-sim stand-in. Nobody is holding the controller: everything reads released / centered.
public class Gamepad {
    public float left_stick_x, left_stick_y, right_stick_x, right_stick_y;
    public float left_trigger, right_trigger;
    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean a, b, x, y;
    public boolean cross, circle, square, triangle;
    public boolean left_bumper, right_bumper;
    public boolean left_stick_button, right_stick_button;
    public boolean guide, start, back, options, share, ps, touchpad;

    public boolean atRest() {
        return left_stick_x == 0 && left_stick_y == 0 && right_stick_x == 0 && right_stick_y == 0
                && left_trigger == 0 && right_trigger == 0;
    }

    public void rumble(int durationMs) {}
    public void rumble(double rumble1, double rumble2, int durationMs) {}
    public void stopRumble() {}
    public boolean isRumbling() { return false; }
    public void setLedColor(double r, double g, double b, int durationMs) {}
}
