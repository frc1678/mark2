package com.team1678.frc2021.auto;

import java.util.List;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.commands.SpinUpCommand;
import com.team1678.frc2021.commands.TuckCommand;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeftSixNearMode extends SequentialCommandGroup{

    public LeftSixNearMode(Swerve s_Swerve) {

        final Intake mIntake = Intake.getInstance();
        final Superstructure mSuperstructure = Superstructure.getInstance();

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory leftSixFirstShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0)),
                List.of(),
                new Pose2d(4.8, 7.5 , Rotation2d.fromDegrees(0.0)),
                Constants.AutoConstants.defaultConfig);

        Trajectory leftSixIntake =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.3, 7.5 , Rotation2d.fromDegrees(0.0)),
                List.of(),
                new Pose2d(8.5, 7.5, Rotation2d.fromDegrees(0.0)),
                Constants.AutoConstants.defaultConfig);

        Trajectory leftSixSecondShot =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(8.5, 7.5 , Rotation2d.fromDegrees(180.0)),
                List.of(),
                new Pose2d(1.3, 7.0, Rotation2d.fromDegrees(180.0)),
                Constants.AutoConstants.defaultConfig);

        SwerveControllerCommand leftSixFirstShotCommand =
            new SwerveControllerCommand(
                leftSixFirstShot,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand leftSixIntakeCommand =
            new SwerveControllerCommand(
                leftSixIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand leftSixSecondShotCommand =
            new SwerveControllerCommand(
                leftSixSecondShot,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);

        IntakeCommand intake = 
            new IntakeCommand(mIntake, mSuperstructure);

        SpinUpCommand spinUp = 
            new SpinUpCommand(mSuperstructure);
            
        ShootCommand firstShoot =
            new ShootCommand(mSuperstructure);

        ShootCommand secondShoot =
            new ShootCommand(mSuperstructure);

        AutoAimCommand firstAim =
            new AutoAimCommand(mSuperstructure, 200);

        AutoAimCommand secondAim =
            new AutoAimCommand(mSuperstructure, 200);

        TuckCommand firstTuck =
            new TuckCommand(mSuperstructure, true);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(leftSixFirstShot.getInitialPose())),
            new SequentialCommandGroup(
                leftSixFirstShotCommand.deadlineWith(new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    spinUp, 
                    firstAim
                )),
                firstShoot,
                firstTuck,
                leftSixIntakeCommand,
                leftSixSecondShotCommand.deadlineWith(new SequentialCommandGroup(
                    new WaitCommand(1.0),
                    (secondAim)
                )),
                secondShoot
            ).deadlineWith(intake)
        );
    }
}
