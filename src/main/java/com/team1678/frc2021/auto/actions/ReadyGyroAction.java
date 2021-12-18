package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;

public class ReadyGyroAction implements Action {

    private final Swerve mSwerve;
    private double adjustedGyroRead;

    public ReadyGyroAction(Swerve swerve) {
        mSwerve = swerve;
    }

    @Override
    public void start() {
        adjustedGyroRead = mSwerve.getYaw().getDegrees() - 180;
    }

    @Override
    public void update() {
        mSwerve.zeroGyro(adjustedGyroRead);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {}
}
