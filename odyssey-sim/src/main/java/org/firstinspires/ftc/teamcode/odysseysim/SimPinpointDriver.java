package org.firstinspires.ftc.teamcode.odysseysim;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

// The goBILDA Pinpoint driver API backed by the simulated device. TeamCode's copy and the SDK's
// GoBildaPinpointDriver are both stand-ins that extend this. Getters return what the last update()
// read, like the real driver. Pod setup (offsets, resolution, directions, yaw scalar) is accepted
// and reported back but the simulated pods are always set up correctly.
public abstract class SimPinpointDriver implements HardwareDevice {

    public enum DeviceStatus {
        NOT_READY, READY, CALIBRATING, FAULT_X_POD_NOT_DETECTED, FAULT_Y_POD_NOT_DETECTED,
        FAULT_NO_PODS_DETECTED, FAULT_IMU_RUNAWAY, FAULT_BAD_READ
    }

    public enum EncoderDirection { FORWARD, REVERSED }

    public enum GoBildaOdometryPods { goBILDA_SWINGARM_POD, goBILDA_4_BAR_POD }

    public enum ReadData { ONLY_UPDATE_HEADING }

    public static final float goBILDA_SWINGARM_POD = 13.26291192f;
    public static final float goBILDA_4_BAR_POD = 19.89436789f;

    private final Simulation sim;
    private final SimPinpoint device;
    private double xOffsetMm;
    private double yOffsetMm;
    private double yawScalar = 1;

    protected SimPinpointDriver() {
        sim = Simulation.current();
        device = sim.pinpoint;
    }

    public void update() {
        sim.io(sim.pinpointUpdateMs);
        device.snapshot();
    }

    public void update(ReadData data) {
        update();
    }

    public void setOffsets(double xOffset, double yOffset) {
        setOffsets(xOffset, yOffset, DistanceUnit.MM);
    }

    public void setOffsets(double xOffset, double yOffset, DistanceUnit distanceUnit) {
        sim.io(2 * sim.pinpointWriteMs);
        xOffsetMm = distanceUnit.toMm(xOffset);
        yOffsetMm = distanceUnit.toMm(yOffset);
    }

    public void recalibrateIMU() {
        sim.io(sim.pinpointWriteMs);
    }

    public void resetPosAndIMU() {
        sim.io(sim.pinpointWriteMs);
        device.reset();
    }

    public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
        sim.io(2 * sim.pinpointWriteMs);
    }

    public void setEncoderResolution(GoBildaOdometryPods pods) {
        sim.io(sim.pinpointWriteMs);
    }

    public void setEncoderResolution(double ticksPerMm) {
        sim.io(sim.pinpointWriteMs);
    }

    public void setEncoderResolution(double ticksPerUnit, DistanceUnit distanceUnit) {
        sim.io(sim.pinpointWriteMs);
    }

    public void setYawScalar(double yawScalar) {
        sim.io(sim.pinpointWriteMs);
        this.yawScalar = yawScalar;
    }

    public Pose2D setPosition(Pose2D pos) {
        sim.io(3 * sim.pinpointWriteMs);
        device.setX(pos.getX(DistanceUnit.MM));
        device.setY(pos.getY(DistanceUnit.MM));
        device.setHeading(pos.getHeading(AngleUnit.RADIANS));
        return pos;
    }

    public void setPosX(double posX, DistanceUnit distanceUnit) {
        sim.io(sim.pinpointWriteMs);
        device.setX(distanceUnit.toMm(posX));
    }

    public void setPosY(double posY, DistanceUnit distanceUnit) {
        sim.io(sim.pinpointWriteMs);
        device.setY(distanceUnit.toMm(posY));
    }

    public void setHeading(double heading, AngleUnit angleUnit) {
        sim.io(sim.pinpointWriteMs);
        device.setHeading(angleUnit.toRadians(heading));
    }

    public int getDeviceID() { return 1; }
    public int getDeviceVersion() { return 2; }
    public float getYawScalar() { return (float) yawScalar; }
    public DeviceStatus getDeviceStatus() { return DeviceStatus.READY; }
    public int getLoopTime() { return 670; }
    public double getFrequency() { return 1e6 / getLoopTime(); }
    public float getXOffset(DistanceUnit distanceUnit) { return (float) distanceUnit.fromMm(xOffsetMm); }
    public float getYOffset(DistanceUnit distanceUnit) { return (float) distanceUnit.fromMm(yOffsetMm); }

    public double getPosX() { return device.readX; }
    public double getPosX(DistanceUnit distanceUnit) { return distanceUnit.fromMm(device.readX); }
    public double getPosY() { return device.readY; }
    public double getPosY(DistanceUnit distanceUnit) { return distanceUnit.fromMm(device.readY); }
    public double getHeading() { return device.readH; }
    public double getHeading(AngleUnit angleUnit) { return angleUnit.fromRadians(device.readH); }
    public double getHeading(UnnormalizedAngleUnit unit) { return unit.fromRadians(device.readH); }
    public double getVelX() { return device.readVelX; }
    public double getVelX(DistanceUnit distanceUnit) { return distanceUnit.fromMm(device.readVelX); }
    public double getVelY() { return device.readVelY; }
    public double getVelY(DistanceUnit distanceUnit) { return distanceUnit.fromMm(device.readVelY); }
    public double getHeadingVelocity() { return device.readVelH; }
    public double getHeadingVelocity(UnnormalizedAngleUnit unit) { return unit.fromRadians(device.readVelH); }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.MM, device.readX, device.readY, AngleUnit.RADIANS, AngleUnit.normalizeRadians(device.readH));
    }

    public Pose2D getVelocity() {
        return new Pose2D(DistanceUnit.MM, device.readVelX, device.readVelY, AngleUnit.RADIANS, device.readVelH);
    }

    @Override
    public String getDeviceName() {
        return "odyssey-sim goBILDA Pinpoint";
    }
}
