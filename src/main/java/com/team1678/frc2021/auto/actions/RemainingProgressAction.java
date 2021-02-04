package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;

/**
 * An action designed to wait until there is only a certain (specified) 
 * amount of time left before the completion of a trajectory.
 */
public class RemainingProgressAction implements Action{
    Swerve mSwerve;
    double targetProgress = 0.0;

    public RemainingProgressAction(double targetProgress){
        mSwerve = Swerve.getInstance();
        this.targetProgress = targetProgress;
    }

    @Override
    public boolean isFinished() {
        return mSwerve.getRemainingProgress() <= targetProgress;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }
}
