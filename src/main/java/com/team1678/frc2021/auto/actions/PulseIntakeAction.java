package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;

public class PulseIntakeAction implements Action {

    private final Intake mIntake;
    private final Superstructure mSuperstructure;
    private boolean isIntaking = false;
    private double mStartTime;

    public PulseIntakeAction(Intake intake, Superstructure superstructure) {
        mIntake = intake;
        mSuperstructure = superstructure; 
    }

    @Override
    public void start() {}

    @Override
    public void update() {
        mSuperstructure.enableIndexer(true);
        if(Timer.getFPGATimestamp() - mStartTime > 0.1){
            if(isIntaking){
                mIntake.setState(Intake.WantedAction.NONE);
                isIntaking = false;
            } else{
                mIntake.setState(Intake.WantedAction.INTAKE);
                isIntaking = true;

            }
            mStartTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {}
}
