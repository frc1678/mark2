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
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        mSuperstructure.enableIndexer(true);
        if(Timer.getFPGATimestamp() - mStartTime > 0.5){
            if(isIntaking){
                mIntake.setState(Intake.WantedAction.RETRACT);
            } else{
                mIntake.setState(Intake.WantedAction.INTAKE);

            }
            mStartTime = Timer.getFPGATimestamp();
        }
    }


}
