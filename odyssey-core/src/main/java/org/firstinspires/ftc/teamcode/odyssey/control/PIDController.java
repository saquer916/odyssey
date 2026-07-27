package org.firstinspires.ftc.teamcode.odyssey.control;

public class PIDController {
    private double setPoint;
    private double kP,kI,kD;

        private double minLimit = Double.NaN ,maxLimit = Double.NaN;

    private double previousTime = Double.NaN;
    private double lastError = 0;
    private double integralError = 0;
    private double maxIntegral = Double.NaN;

        public PIDController(final double setPoint, final double kP, final double kI, final double kD) {
        this.setSetpoint(setPoint);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getOutput(final double currentTime, final double currentValue) {
        final double error = setPoint - currentValue;
        final double dt = !Double.isNaN(previousTime) ?
                (currentTime - previousTime) : 0;

        final double derivativeError = (dt != 0) ? ((error - lastError) / dt) : 0;

        integralError += error * dt;

        if (!Double.isNaN(maxIntegral) && kI != 0) {
            double integralContribution = kI * integralError;
            if (integralContribution > maxIntegral) {
                integralError = maxIntegral / kI;
            } else if (integralContribution < -maxIntegral) {
                integralError = -maxIntegral / kI;
            }
        }

        previousTime = currentTime;
        lastError = error;

        return checkLimits((kP * error) + (kI * integralError) + (kD * derivativeError));
    }

    public void reset() {
        previousTime = Double.NaN;
        lastError = 0;
        integralError = 0;
    }

    private void resetErrors() {
            lastError = 0;
            integralError = 0;
    }

    private double checkLimits(final double output){
        if (!Double.isNaN(minLimit) && output < minLimit)
            return minLimit;
        else if (!Double.isNaN(maxLimit) && output > maxLimit)
            return maxLimit;
        else
            return output;
    }

    public void setOutputLimits(final double minLimit, final double maxLimit) {
        if(minLimit < maxLimit) {
            this.minLimit = minLimit;
            this.maxLimit = maxLimit;
        }else{
            this.minLimit = maxLimit;
            this.maxLimit = minLimit;
        }
    }

    public void removeOutputLimits() {
        this.minLimit = Double.NaN;
        this.maxLimit = Double.NaN;
    }
    public double getkP() {
        return kP;
    }
    public void setkP(double kP) {
        this.kP = kP;
        resetErrors();
    }
    public double getkI() {
        return kI;
    }
    public void setkI(double kI) {
        this.kI = kI;
        resetErrors();
    }
    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
        resetErrors();
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetpoint(final double setPoint) {
        resetErrors();
        this.setPoint = setPoint;
    }
    public void setMaxIntegral(final double maxIntegral) {
        this.maxIntegral = Math.abs(maxIntegral);
    }
}