package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Shooter;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitToSpinUpCommand extends CommandBase {

    private final Superstructure mSuperstructure;
    private final Shooter mShooter;
    private boolean isFinished = false;

    private double mTimeToWait;
    private double mStartTime;

    public WaitToSpinUpCommand(Superstructure superstructure, double waitTime) {
        mSuperstructure = superstructure;
        mTimeToWait = waitTime;
        mShooter = Shooter.getInstance();
    }

    @Override 
    public void initialize() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - mStartTime >= mTimeToWait) {
            mSuperstructure.setWantSpinUp(true);
            return true;
        }
        return false;
    }

}