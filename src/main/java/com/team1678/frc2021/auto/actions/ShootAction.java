package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Trigger;

import edu.wpi.first.wpilibj.Timer;

public class ShootAction implements Action {

    private final Superstructure mSuperstructure;
    private double startTime = Double.POSITIVE_INFINITY;
    private boolean isFinished = false;

    public ShootAction(Superstructure superstructure) {
        mSuperstructure = superstructure;
    }

    @Override
    public void start() {}

    @Override
    public void update() {
        if (Trigger.getInstance().getPopoutSolenoid() && startTime == Double.POSITIVE_INFINITY) {
            startTime = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - startTime > 1.5) {
            mSuperstructure.setWantShoot(false);
            mSuperstructure.setWantSpinUp(true);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {}
}
