package com.team1678.lib.control;

/**
 * PID controller that controls a control loop
 */

public class PIDController {

    private double kP;
    private double kI;
    private double kD;
    private double setpoint = 0.0;
 
    private double prevError = 0.0;
    private double integral = 0.0;

    private double prevTime = 0.0;
    private double kDeadband = 0.0;
    
    public PIDController(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public PIDController(double p, double i, double d, double deadband) {
        kP = p;
        kI = i;
        kD = d;
        kDeadband = deadband;
    }

    public void reset() {
        prevError = 0;
        prevTime = 0;
        integral = 0;

    }

    public double update(double timestamp, double sensor) {
        double dt = timestamp - prevTime;
        
        double error = setpoint - sensor;

        prevTime = timestamp;

        if (Math.abs(error) < kDeadband) {
            return 0.0;
        }
        return (kP * error) + (kI * calculateIntegral(dt, error)) + (kD * calculateDerivative(dt, error));
    }

    public void setGoal(double setpoint) {
        this.setpoint = setpoint;
    }

    // calculate functions could have been in the update lol

    private double calculateDerivative(double dt, double error) {
        double derivative = (error - prevError) / dt;
        prevError = error;
        return derivative;
    }

    private double calculateIntegral(double dt, double error) {
        integral += error * dt;
      //  prevError = error;
        return integral;
    }
}