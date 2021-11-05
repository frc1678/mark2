package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitToAutoAimCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final double mTurretAngle;

    private double mStartTime;
    private double mTimeToWait;

    public WaitToAutoAimCommand(Superstructure superstructure, double turretAngle, double waitTime) {
        mTimeToWait = waitTime;
        mSuperstructure = superstructure;
        mTurretAngle = turretAngle;
    }

    @Override
    public void initialize() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override 
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - mStartTime >= mTimeToWait) {
            mSuperstructure.setWantTuck(false);
            mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(mTurretAngle));
            return true;
        }
        return false;
    }
}