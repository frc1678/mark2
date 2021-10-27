package com.team1678.frc2021.auto;

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

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.commands.SpinUpCommand;
import com.team1678.frc2021.commands.TuckCommand;
import com.team1678.frc2021.subsystems.Indexer;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

public class LeftEightNearMode extends SequentialCommandGroup{

    final Intake mIntake = Intake.getInstance();
    final Indexer mIndexer = Indexer.getInstance();
    final Superstructure mSuperstructure = Superstructure.getInstance();
    
    public LeftEightNearMode(Swerve s_Swerve){

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory leftEightFirstShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(180.0)),
                List.of(),
                new Pose2d(1.7, 5.8 , Rotation2d.fromDegrees(270.0)),
                Constants.AutoConstants.defaultConfig);

        Trajectory leftEightIntake =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.5, 6.0 , Rotation2d.fromDegrees(90.0)),
                List.of(new Translation2d(2.9, 7.0), new Translation2d(5.0, 7.0)),
                new Pose2d(9.6, 7.3, Rotation2d.fromDegrees(0.0)),
                Constants.AutoConstants.defaultConfig);

        Trajectory leftEightSecondShot =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(9.6, 7.5 , Rotation2d.fromDegrees(180.0)),
                List.of(new Translation2d(2.9, 7.0)),
                new Pose2d(1.7, 5.8, Rotation2d.fromDegrees(270.0)),
                Constants.AutoConstants.defaultConfig);
    
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

        IntakeCommand intake = 
            new IntakeCommand(mIntake, mSuperstructure);

        SpinUpCommand spinUp = 
            new SpinUpCommand(mSuperstructure);
            
        ShootCommand shoot =
            new ShootCommand(mSuperstructure);

        AutoAimCommand aim =
            new AutoAimCommand(mSuperstructure, 180);

        TuckCommand firstTuck =
            new TuckCommand(mSuperstructure, true);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0)))),
            leftEightFirstShotCommand.deadlineWith(aim, spinUp, intake),
            shoot,
            new WaitCommand(1.0),
            firstTuck,
            leftEightIntakeCommand,
            leftEightSecondShotCommand
            //shoot
        );

    }
    
}
