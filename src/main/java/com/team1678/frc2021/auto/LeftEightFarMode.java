package com.team1678.frc2021.auto;

import java.util.List;

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

import com.team1678.frc2021.commands.WaitAfterDrive;
import com.team1678.frc2021.commands.WaitToAutoAimCommand;
import com.team1678.frc2021.commands.WaitToIntakeCommand;
import com.team1678.frc2021.commands.WaitToSpinUpCommand;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.commands.SpinUpCommand;
import com.team1678.frc2021.commands.SwervePointTurnCommand;
import com.team1678.frc2021.commands.TuckCommand;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;

public class LeftEightFarMode extends SequentialCommandGroup{
    
    public LeftEightFarMode(Swerve s_Swerve){

        final Intake mIntake = Intake.getInstance();
        final Superstructure mSuperstructure = Superstructure.getInstance();

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory leftEightFirstShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0)),
                List.of(),
                new Pose2d(4.8, 7.5 , Rotation2d.fromDegrees(0.0)),
                Constants.AutoConstants.defaultConfig);

        Trajectory leftEightIntake =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.3, 7.5 , Rotation2d.fromDegrees(0.0)),
                List.of(),
                new Pose2d(10.0, 7.5, Rotation2d.fromDegrees(0.0)),
                Constants.AutoConstants.defaultConfig);

        Trajectory leftEightSecondShot =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(10.0, 7.5 , Rotation2d.fromDegrees(180.0)),
                List.of(),
                new Pose2d(5.3, 7.5, Rotation2d.fromDegrees(180.0)),
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

        SwervePointTurnCommand endAdjustCommand =
            new SwervePointTurnCommand(
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(180),
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
    
        TuckCommand secondTuck =
            new TuckCommand(mSuperstructure, false);

        WaitToSpinUpCommand waitToSpinUp = 
            new WaitToSpinUpCommand(mSuperstructure, 1.5);

        WaitToAutoAimCommand waitToAutoAim = 
            new WaitToAutoAimCommand(mSuperstructure, 200, 1.5);

        WaitToAutoAimCommand secondWaitToAutoAim = 
            new WaitToAutoAimCommand(mSuperstructure, 200, 4.0);

        WaitToIntakeCommand waitToFirstIntake = 
            new WaitToIntakeCommand(mIntake, mSuperstructure, 1.5);

        IntakeCommand secondIntake = 
            new IntakeCommand(mIntake, mSuperstructure);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(leftEightFirstShot.getInitialPose())),
            new SequentialCommandGroup(
                leftEightFirstShotCommand.deadlineWith(
                    // new WaitAfterDrive(5.0),
                    waitToSpinUp, 
                    waitToAutoAim,
                    waitToFirstIntake
                ),
                firstShoot,
                firstTuck.deadlineWith(secondIntake),
                leftEightIntakeCommand,
                leftEightSecondShotCommand,
                secondTuck,
                secondAim,
                secondShoot
                // endAdjustCommand
            )
        );
    }
    
}
