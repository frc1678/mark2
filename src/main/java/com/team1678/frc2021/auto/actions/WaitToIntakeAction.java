package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;

public class WaitToIntakeAction implements Action {
    
    private final Intake mIntake;
    private final Superstructure mSuperstructure;

    private double mTimeToWait;
    private double mStartTime;
    private boolean mWaitFinished;

    public WaitToIntakeAction(Intake intake, Superstructure superstructure, double waitTime) {
        mTimeToWait = waitTime;
        mIntake = intake;
        mSuperstructure = superstructure;
        mWaitFinished = false;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();

    }

    @Override
    public void update() {
        if (Timer.getFPGATimestamp() - mStartTime >= mTimeToWait) {
            mIntake.setState(Intake.WantedAction.INTAKE);
            mSuperstructure.enableIndexer(true);
            mWaitFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return mWaitFinished;
    }

    @Override
    public void done() {}
}
