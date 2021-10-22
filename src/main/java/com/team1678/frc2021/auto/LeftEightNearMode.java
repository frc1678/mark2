package com.team1678.frc2021.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Swerve;

public class LeftEightNearMode extends SequentialCommandGroup{
    
    public LeftEightNearMode(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        TrajectoryConfig endConfig =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);


        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory leftEightFirstShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(180.0)),
                List.of(),
                new Pose2d(1.7, 5.8 , Rotation2d.fromDegrees(270.0)),
                config);

        Trajectory leftEightIntake =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.5, 6.0 , Rotation2d.fromDegrees(90.0)),
                List.of(new Translation2d(2.9, 7.0), new Translation2d(5.0, 7.0)),
                new Pose2d(9.6, 7.3, Rotation2d.fromDegrees(0.0)),
                config);

        Trajectory leftEightSecondShot =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(9.6, 7.5 , Rotation2d.fromDegrees(180.0)),
                List.of(new Translation2d(2.9, 7.0)),
                new Pose2d(1.7, 5.8, Rotation2d.fromDegrees(270.0)),
                config);
    
        SwerveControllerCommand leftEightFirstShotCommand =
            new SwerveControllerCommand(
                leftEightFirstShot,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand leftEightIntakeCommand =
            new SwerveControllerCommand(
                leftEightIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand leftEightSecondShotCommand =
            new SwerveControllerCommand(
                leftEightSecondShot,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0)))),
            leftEightFirstShotCommand,
            leftEightIntakeCommand,
            leftEightSecondShotCommand
        );

    }
    
}
