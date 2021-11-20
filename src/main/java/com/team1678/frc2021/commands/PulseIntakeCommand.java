package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PulseIntakeCommand extends CommandBase{

    private final Intake mIntake;
    private final Superstructure mSuperstructure;
    private boolean isIntaking = false;
    private double mStartTime;

    public PulseIntakeCommand(Intake intake, Superstructure superstructure) {
        mIntake = intake;
        mSuperstructure = superstructure; 
    }

    @Override
    public void initialize(){
        mStartTime = Timer.getFPGATimestamp();
    }
    @Override
    public void execute() {
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


}
