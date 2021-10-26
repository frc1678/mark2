
package com.team1678.frc2021.auto;

import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AimTestAuto extends SequentialCommandGroup {

    public static boolean mIsFinished = false;

    public AimTestAuto(Swerve s_Swerve){
        
        Superstructure mSuperstructure = Superstructure.getInstance();

        AutoAimCommand aim =
            new AutoAimCommand(mSuperstructure, 180);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
            new ParallelCommandGroup(
                aim
            )
        );
    }

}