package com.team1678.frc2021.auto;

import com.team1678.frc2021.Constants;

import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.ReadyGyro;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.commands.SpinUpCommand;
import com.team1678.frc2021.commands.SwervePointTurnCommand;
import com.team1678.frc2021.commands.TuckCommand;
import com.team1678.frc2021.commands.WaitToAutoAimCommand;
import com.team1678.frc2021.commands.WaitToIntakeCommand;
import com.team1678.frc2021.commands.WaitToSpinUpCommand;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightFiveNearMode extends SequentialCommandGroup {

    public RightFiveNearMode(Swerve s_Swerve) {
        final Intake mIntake = Intake.getInstance();
        final Superstructure mSuperstructure = Superstructure.getInstance();

        Trajectory firstIntake =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.90, 0.71, Rotation2d.fromDegrees(0.0)),
                List.of(new Translation2d(6.0, 0.4)),
                new Pose2d(5.76, 0.71, Rotation2d.fromDegrees(135.0)),
                Constants.AutoConstants.zeroToSlow);

        Trajectory closeShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.9, 0.71, Rotation2d.fromDegrees(135.0)),
                List.of(new Translation2d(2.54, 2.54),
                        new Translation2d(1.52, 4.84)),
                new Pose2d(1.52, 5.94, Rotation2d.fromDegrees(90.0)),
                Constants.AutoConstants.slowToZero);

            var thetaController =
                new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand firstIntakeCommand =
            new SwerveControllerCommand(
                firstIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(15), //Swerve Heading
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand closeShotCommand =
            new SwerveControllerCommand(
                closeShot,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(179), // Swerve Heading
                s_Swerve::setModuleStates,
                s_Swerve);

        IntakeCommand intake =
            new IntakeCommand(mIntake, mSuperstructure);

        ShootCommand shoot =
            new ShootCommand(mSuperstructure);

        AutoAimCommand aim =
            new AutoAimCommand(mSuperstructure, 180);

        WaitToSpinUpCommand waitToSpinUp =
            new WaitToSpinUpCommand(mSuperstructure, 0.5);

        WaitToIntakeCommand waitToFirstIntake =
            new WaitToIntakeCommand(mIntake, mSuperstructure, 0.0);

        TuckCommand tuck =
            new TuckCommand(mSuperstructure, true);

        TuckCommand untuck =
            new TuckCommand(mSuperstructure, false);
        
        ReadyGyro readyGyro = 
            new ReadyGyro(s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(2.90, 0.71, Rotation2d.fromDegrees(0.0)))),
            new SequentialCommandGroup(
                firstIntakeCommand.deadlineWith(
                    tuck,
                    waitToSpinUp,
                    waitToFirstIntake
                ),
                closeShotCommand.deadlineWith(intake),
                untuck,
                aim,
                shoot,
                readyGyro
            )
        );
    }
}