package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitToIntakeCommand extends CommandBase{

    private final Intake mIntake;
    private final Superstructure mSuperstructure;

    private double mTimeToWait;
    private double mStartTime;

    public WaitToIntakeCommand(Intake intake, Superstructure superstructure, double waitTime) {
        mTimeToWait = waitTime;
        mIntake = intake;
        mSuperstructure = superstructure;
    }

    @Override 
    public void initialize() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - mStartTime >= mTimeToWait) {
            mIntake.setState(Intake.WantedAction.INTAKE);
            mSuperstructure.enableIndexer(true);
        }
    }


}