package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase{

    private final Intake mIntake;
    private final Superstructure mSuperstructure;

    public IntakeCommand(Intake intake, Superstructure superstructure) {
        mIntake = intake;
        mSuperstructure = superstructure;
    }

    @Override
    public void execute() {
        mIntake.setState(Intake.WantedAction.INTAKE);
        mSuperstructure.enableIndexer(true);
    }


}
