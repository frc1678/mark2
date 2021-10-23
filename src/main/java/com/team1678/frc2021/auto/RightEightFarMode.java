package com.team1678.frc2021.auto;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.commands.SpinUpCommand;
import com.team1678.frc2021.commands.SwervePointTurnCommand;

import com.team1678.frc2021.subsystems.Indexer;
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
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class RightEightFarMode extends ParallelCommandGroup {

    public RightEightFarMode(Swerve s_Swerve){

        final Intake mIntake = Intake.getInstance();
        final Indexer mIndexer = Indexer.getInstance();
        final Superstructure mSuperstructure = Superstructure.getInstance();

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        TrajectoryConfig slowConfig = 
            new TrajectoryConfig(
                Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

        TrajectoryConfig shotToIntakeConfig =
            new TrajectoryConfig(
                    Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);

        shotToIntakeConfig.setStartVelocity(0);

        shotToIntakeConfig.setEndVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);

        TrajectoryConfig firstIntakeConfig =
        new TrajectoryConfig(
            Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        firstIntakeConfig.setStartVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);
        firstIntakeConfig.setEndVelocity(0);
        
        TrajectoryConfig headingAdjustConfig =
        new TrajectoryConfig(
            Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        headingAdjustConfig.setStartVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);
        headingAdjustConfig.setEndVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);

        TrajectoryConfig secondIntakeConfig =
        new TrajectoryConfig(
            Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        secondIntakeConfig.setStartVelocity(0.0);
        secondIntakeConfig.setEndVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);
    
        TrajectoryConfig secondShotConfig =
        new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);        
        secondShotConfig.setStartVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);
        secondShotConfig.setEndVelocity(0);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory firstShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.90, 0.71, Rotation2d.fromDegrees(0.0)),
                List.of(new Translation2d(5.0, 0.71),
                        new Translation2d(6.0, 1.0),
                        new Translation2d(4.5, 4.0)),
                new Pose2d(4.0, 6.0, Rotation2d.fromDegrees(90.0)), 
                config);

        Trajectory shotToIntake =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(4.0, 6.0, Rotation2d.fromDegrees(0.0)),
                List.of(new Translation2d(5.0, 6.5)),
                new Pose2d(6.8, 6.0, Rotation2d.fromDegrees(260.0)), 
                shotToIntakeConfig);
        
        Trajectory firstIntake =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(6.8, 6.0, Rotation2d.fromDegrees(260.0)),
                List.of(),
                new Pose2d(6.45, 4.7, Rotation2d.fromDegrees(260.0)), 
                firstIntakeConfig);

        Trajectory headingAdjust =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(6.45, 4.8, Rotation2d.fromDegrees(110)),
                List.of(),
                new Pose2d(6.45, 4.85, Rotation2d.fromDegrees(110)), 
                headingAdjustConfig);
        
        Trajectory secondIntake =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(6.45, 4.85, Rotation2d.fromDegrees(110)),
                List.of(),
                new Pose2d(5.0, 6.5, Rotation2d.fromDegrees(180)), 
                secondIntakeConfig);

        Trajectory secondShoot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.0, 6.5, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(4.0, 6.0, Rotation2d.fromDegrees(225)), 
                secondShotConfig);
        // Trajectory secondIntake =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(6.45, 4.4, Rotation2d.fromDegrees(100.0)),
        //         List.of(new Translation2d(5.0, 6.5)),
        //         new Pose2d(4.0, 6.0, Rotation2d.fromDegrees(180.0)), 
        //         config);          
    
        SwerveControllerCommand firstShotCommand =
            new SwerveControllerCommand(
                firstShot,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(45),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand shotToIntakeCommand =
            new SwerveControllerCommand(
                shotToIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(260),
                s_Swerve::setModuleStates,
                s_Swerve);
        
        SwerveControllerCommand firstIntakeCommand =
            new SwerveControllerCommand(
                firstIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(260),
                s_Swerve::setModuleStates,
                s_Swerve);
/*
        SwerveControllerCommand headingAdjustCommand =
            new SwerveControllerCommand(
                headingAdjust,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(110),
                s_Swerve::setModuleStates,
                s_Swerve);
                */

        SwervePointTurnCommand headingAdjustCommand =
            new SwervePointTurnCommand(
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(140),
                s_Swerve::setModuleStates,
                s_Swerve);
            
        SwerveControllerCommand secondIntakeCommand =
            new SwerveControllerCommand(
                secondIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand sceondShotCommand =
            new SwerveControllerCommand(
                secondShoot,
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
            
        ShootCommand shoot =
            new ShootCommand(mSuperstructure);

        AutoAimCommand aim =
            new AutoAimCommand(mSuperstructure, 120);
        
        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(firstShot.getInitialPose())),
            firstShotCommand,
            shoot,
            shotToIntakeCommand,
            firstIntakeCommand,
            headingAdjustCommand,
            secondIntakeCommand,
            sceondShotCommand,
            shoot
        );

        addCommands(spinUp);
        addCommands(intake);
        addCommands(aim);
    }
}