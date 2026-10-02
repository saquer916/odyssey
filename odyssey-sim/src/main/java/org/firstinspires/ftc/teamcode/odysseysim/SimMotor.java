package org.firstinspires.ftc.teamcode.odysseysim;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// One drive motor, behaving like the SDK's motor on a REV hub where it matters for driving:
// direction is applied when power is sent, an unchanged power isn't re-sent (no bus time), NaN
// arrives as 0, and the encoder follows the wheel.
final class SimMotor implements DcMotorEx {
    private final Simulation sim;
    private final int index;
    final String name;
    private final double mount;
    private final double ticksPerMeter;
    private final double wheelRadius;

    private Direction direction = Direction.FORWARD;
    private ZeroPowerBehavior zeroPowerBehavior;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private double power;       // what the OpMode last set (clipped), in its own sense
    private double sentPower;   // what the hub is applying, motor-shaft sense
    private boolean enabled = true;
    private int targetPosition;
    private double encoderZero;

    SimMotor(Simulation sim, int index, String name, double mount, SimConfig config) {
        this.sim = sim;
        this.index = index;
        this.name = name;
        this.mount = mount;
        this.wheelRadius = config.number("wheel.radiusMm") / 1000.0;
        this.ticksPerMeter = config.number("motor.ticksPerRev") / (2 * Math.PI * wheelRadius);
        this.zeroPowerBehavior = ZeroPowerBehavior.valueOf(config.string("motor.defaultZeroPowerBehavior"));
    }

    // Fraction of battery voltage on this wheel, in the wheel's forward sense.
    double wheelPower() {
        return enabled ? sentPower * mount : 0;
    }

    boolean braking() {
        return enabled && zeroPowerBehavior != ZeroPowerBehavior.FLOAT;
    }

    // getPower() without the bus read, for recording.
    double requestedPower() {
        return power;
    }

    // The SDK zeroes every motor when an OpMode ends, without the OpMode doing any I/O.
    void forceStop() {
        power = 0;
        sentPower = 0;
    }

    private double directionSign() {
        return direction == Direction.REVERSE ? -1 : 1;
    }

    private double rawTicks() {
        return sim.physics.wheelTravel[index] * mount * ticksPerMeter;
    }

    @Override
    public void setPower(double power) {
        if (Double.isNaN(power)) {
            sim.recordNaNPower(name);
            power = 0;
        }
        power = Range.clip(power, -1, 1);
        this.power = power;
        double toSend = power * directionSign();
        if (toSend != sentPower) {
            sim.io(sim.motorWriteMs);
            sentPower = toSend;
        }
    }

    @Override
    public double getPower() {
        sim.io(sim.motorReadMs);
        return power;
    }

    @Override public void setDirection(Direction direction) { this.direction = direction; }
    @Override public Direction getDirection() { return direction; }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        if (zeroPowerBehavior == ZeroPowerBehavior.UNKNOWN) return;
        if (zeroPowerBehavior != this.zeroPowerBehavior) sim.io(sim.motorWriteMs);
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }

    @Override
    public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0);
    }

    @Override
    public boolean getPowerFloat() {
        return zeroPowerBehavior == ZeroPowerBehavior.FLOAT && power == 0;
    }

    @Override
    public void setMode(RunMode mode) {
        sim.io(sim.motorWriteMs);
        switch (mode) {
            case RUN_TO_POSITION:
                throw new UnsupportedOperationException("odyssey-sim: RUN_TO_POSITION on \"" + name
                        + "\" needs the hub's position PID, which isn't simulated");
            case RUN_USING_ENCODER:
                sim.warn("\"" + name + "\" is in RUN_USING_ENCODER; the hub's velocity PID isn't simulated, "
                        + "so power is applied open-loop");
                break;
            case STOP_AND_RESET_ENCODER:
                encoderZero = rawTicks();
                forceStop();
                break;
            default:
                break;
        }
        this.mode = mode;
    }

    @Override public RunMode getMode() { return mode; }

    @Override
    public int getCurrentPosition() {
        sim.io(sim.motorReadMs);
        return (int) Math.round((rawTicks() - encoderZero) * directionSign());
    }

    @Override
    public double getVelocity() {
        sim.io(sim.motorReadMs);
        return sim.physics.wheelSpeed(index) * mount * ticksPerMeter * directionSign();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        double radiansPerSecond = getVelocity() / ticksPerMeter / wheelRadius;
        return unit == AngleUnit.DEGREES ? Math.toDegrees(radiansPerSecond) : radiansPerSecond;
    }

    @Override
    public void setVelocity(double angularRate) {
        throw new UnsupportedOperationException("odyssey-sim: setVelocity on \"" + name
                + "\" needs the hub's velocity PID, which isn't simulated; use setPower");
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        setVelocity(angularRate);
    }

    @Override public void setTargetPosition(int position) { targetPosition = position; }
    @Override public int getTargetPosition() { return targetPosition; }
    @Override public boolean isBusy() { return false; }
    @Override public int getPortNumber() { return index; }

    @Override
    public void setMotorEnable() {
        sim.io(sim.motorWriteMs);
        enabled = true;
    }

    @Override
    public void setMotorDisable() {
        sim.io(sim.motorWriteMs);
        enabled = false;
    }

    @Override public boolean isMotorEnabled() { return enabled; }
    @Override public String getDeviceName() { return "odyssey-sim drive motor"; }
}
