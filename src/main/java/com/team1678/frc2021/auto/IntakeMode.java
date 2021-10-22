package com.team1678.frc2021.auto;

import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeMode extends SequentialCommandGroup {

    public IntakeMode(Swerve mSwerve){
        
        final Intake mIntake = Intake.getInstance();

        IntakeCommand intake = 
            new IntakeCommand(mIntake);

        addCommands(intake);
    }

    
}
