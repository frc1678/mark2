package com.team1678.lib.control;

public class StabilizingController {
    private PIDController mController;
    private double mSetpoint = 0; // what setpoint to stabilize around
    private double mRate = 0; // goal "velocity" we want setpoint to change at
    private double mPrevTime = 0;

    public StabilizingController(double p, double i, double d) {
        mController = new PIDController(p, i, d);
    }

    public void setRate(double rate) {
        mRate = rate;
    }

    public final double rate() {
        return mRate;
    }

    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
    }

    public final double setpoint() {
        return mSetpoint;
    }

    public double update(double timestamp, double current_pos) {
        if (mPrevTime == 0) {
            mPrevTime = timestamp;
            return 0.0;
        }
        double dt = timestamp - mPrevTime;
        mSetpoint += dt * mRate;
        mController.setGoal(mSetpoint);
        return mController.update(timestamp, current_pos);
    }
}