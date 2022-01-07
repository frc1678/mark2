package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

public class AutoAimAction implements Action {

    private final Superstructure mSuperstructure;
    private final double mTurretAngle;

    public AutoAimAction(Superstructure superstructure, double turretAngle) {
        mSuperstructure = superstructure;
        mTurretAngle = turretAngle;
    }
    
    @Override
    public void start() {
        mSuperstructure.setWantTuck(false);
        mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(mTurretAngle));
    }

    @Override
    public void update() {
        mSuperstructure.setWantTuck(false);
        mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(mTurretAngle));
    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.isAutoAiming() && Math.abs(mSuperstructure.mFieldRelativeTurretGoal.getDegrees() - mTurretAngle) < 10;
    }

    @Override
    public void done() {}
}
