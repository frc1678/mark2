package com.team1678.frc2021.auto;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.commands.SpinUpCommand;
import com.team1678.frc2021.commands.SwervePointTurnCommand;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightTenFarMode extends SequentialCommandGroup {

    public RightTenFarMode(Swerve s_Swerve){

        final Intake mIntake = Intake.getInstance();
        final Superstructure mSuperstructure = Superstructure.getInstance();

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory getTofirstShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.90, 0.71, Rotation2d.fromDegrees(0.0)),
                List.of(new Translation2d(5.0, 0.71),
                        new Translation2d(6.3, 0.9),
                        new Translation2d(4.5, 4.0)),
                new Pose2d(4.2, 6.0, Rotation2d.fromDegrees(90.0)), 
                Constants.AutoConstants.defaultConfig);

        Trajectory getToShieldGen =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(4.2, 6.0, Rotation2d.fromDegrees(0.0)),
                List.of(/*new Translation2d(5.0, 6.5)*/),
                new Pose2d(7.2, 6.0, Rotation2d.fromDegrees(260.0)), 
                Constants.AutoConstants.zeroToSlow);
        
        Trajectory getToFirstIntake =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(7.2, 6.0, Rotation2d.fromDegrees(260.0)),
                List.of(),
                new Pose2d(7.0, 4.85, Rotation2d.fromDegrees(260.0)), 
                Constants.AutoConstants.slowToZero);

        Trajectory getToSecondIntake =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(6.6, 4.85, Rotation2d.fromDegrees(110)),
                List.of(),
                new Pose2d(6.0, 6.5, Rotation2d.fromDegrees(180)), 
                Constants.AutoConstants.zeroToSlow);

        Trajectory getToSecondShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(6.0, 6.5, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(4.5, 6.0, Rotation2d.fromDegrees(225)), 
                Constants.AutoConstants.slowToZero);

        SwerveControllerCommand driveToFirstShotCommand =
            new SwerveControllerCommand(
                getTofirstShot,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(15),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand driveToShieldGenerator =
            new SwerveControllerCommand(
                getToShieldGen,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(260),
                s_Swerve::setModuleStates,
                s_Swerve);
        
        SwerveControllerCommand driveFirstIntakeCommand =
            new SwerveControllerCommand(
                getToFirstIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(260),
                s_Swerve::setModuleStates,
                s_Swerve);

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
            
        SwerveControllerCommand driveSecondIntakeCommand =
            new SwerveControllerCommand(
                getToSecondIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(0),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand driveToSceondShotCommand =
            new SwerveControllerCommand(
                getToSecondShot,
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

        SpinUpCommand firstSpinUp = 
            new SpinUpCommand(mSuperstructure);

        SpinUpCommand secondSpinUp = 
            new SpinUpCommand(mSuperstructure);
            
        ShootCommand firstShoot =
            new ShootCommand(mSuperstructure);

        ShootCommand secondShoot =
            new ShootCommand(mSuperstructure);

        AutoAimCommand aim =
            new AutoAimCommand(mSuperstructure, 180);
        
        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(getTofirstShot.getInitialPose())),
            new SequentialCommandGroup(
                driveToFirstShotCommand.deadlineWith(new SequentialCommandGroup(
                    new WaitCommand(1.0),
                    aim,
                    firstSpinUp
                )),
                firstShoot,
                driveToShieldGenerator.deadlineWith(secondSpinUp),
                driveFirstIntakeCommand,
                headingAdjustCommand,
                driveSecondIntakeCommand,
                driveToSceondShotCommand,
                secondShoot
            ).deadlineWith(intake)
        );
    }
}