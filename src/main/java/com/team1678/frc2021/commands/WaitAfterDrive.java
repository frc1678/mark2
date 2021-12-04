package com.team1678.frc2021.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitAfterDrive extends CommandBase {

    private double mStartTime;
    private double mTimeToWait;

    public WaitAfterDrive(double waitTime) {
        mTimeToWait = waitTime;
    }

    @Override 
    public void initialize() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - mStartTime >= mTimeToWait) {
            return true;
        }
        return false;
    }

}