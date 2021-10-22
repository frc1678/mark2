package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Intake.WantedAction;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase{

    private final Intake mIntake;

    public IntakeCommand(Intake intake) {
        mIntake = intake;
    }

    @Override
    public void execute() {
        mIntake.setState(WantedAction.INTAKE);
    }


}
