
package com.team1678.frc2021.auto;

import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.SpinUpCommand;
import com.team1678.frc2021.commands.WaitAfterDrive;
import com.team1678.frc2021.commands.WaitToSpinUpCommand;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AimTestAuto extends SequentialCommandGroup {

    public static boolean mIsFinished = false;

    public AimTestAuto(Swerve s_Swerve){

        Intake mIntake = Intake.getInstance();
        Superstructure mSuperstructure = Superstructure.getInstance();

        AutoAimCommand aim =
            new AutoAimCommand(mSuperstructure, 180);

        WaitAfterDrive waitAfterDrive = 
            new WaitAfterDrive(4.0);

        IntakeCommand intake = 
            new IntakeCommand(mIntake, mSuperstructure);

        SpinUpCommand spinUp = 
            new SpinUpCommand(mSuperstructure);


        AutoAimCommand firstAim =
            new AutoAimCommand(mSuperstructure, 200);

        WaitToSpinUpCommand waitToSpinUp = 
            new WaitToSpinUpCommand(mSuperstructure, 4.0);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
            waitAfterDrive,
            new ParallelCommandGroup(
                spinUp,
                intake,
                firstAim
            )
        );
    }

}