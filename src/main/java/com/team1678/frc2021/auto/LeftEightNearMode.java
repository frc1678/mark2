package com.team1678.frc2021.auto;

import java.util.List;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.PulseIntakeCommand;
import com.team1678.frc2021.commands.ReadyGyro;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.commands.TuckCommand;
import com.team1678.frc2021.commands.WaitToAutoAimCommand;
import com.team1678.frc2021.commands.WaitToIntakeCommand;
import com.team1678.frc2021.commands.WaitToSpinUpCommand;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

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

public class LeftEightNearMode extends SequentialCommandGroup{
    
    public LeftEightNearMode(Swerve s_Swerve){

        final Intake mIntake = Intake.getInstance();
        final Superstructure mSuperstructure = Superstructure.getInstance();

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory leftSixFirstShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(180.0)),
                List.of(new Translation2d(2.5, 7.3)),
                new Pose2d(1.52, 5.84, Rotation2d.fromDegrees(270.0)),
                Constants.AutoConstants.fastConfig);

        Trajectory leftSixIntake =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.52, 5.84, Rotation2d.fromDegrees(90.0)),
                List.of(new Translation2d(2.9, 7.0)),
                new Pose2d(10.0, 7.22, Rotation2d.fromDegrees(0.0)),
                Constants.AutoConstants.fastConfig);

        Trajectory leftSixSecondShot =          
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(10.0, 7.22 , Rotation2d.fromDegrees(180.0)),
                List.of(new Translation2d(2.9, 7.1)),
                new Pose2d(1.7, 5.75, Rotation2d.fromDegrees(270.0)),
                Constants.AutoConstants.fastConfig);
    
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

        SwerveControllerCommand leftEightIntakeCommand =
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

        SwerveControllerCommand leftEightSecondShotCommand =
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
            
        ShootCommand firstShoot =
            new ShootCommand(mSuperstructure);

        ShootCommand secondShoot =
            new ShootCommand(mSuperstructure);

        WaitToAutoAimCommand secondAim =
            new WaitToAutoAimCommand(mSuperstructure, 200, 2.0);

        TuckCommand tuck =
            new TuckCommand(mSuperstructure, true);

        PulseIntakeCommand unjam = 
            new PulseIntakeCommand(mIntake, mSuperstructure);

        WaitToSpinUpCommand waitToSpinUp = 
            new WaitToSpinUpCommand(mSuperstructure, 1.5);

        WaitToAutoAimCommand waitToAutoAim = 
            new WaitToAutoAimCommand(mSuperstructure, 200, 1.0);

        WaitToIntakeCommand waitToFirstIntake = 
            new WaitToIntakeCommand(mIntake, mSuperstructure, 1.5);

        ReadyGyro readyGyro = 
            new ReadyGyro(s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0)))),
            new SequentialCommandGroup(
                leftSixFirstShotCommand.deadlineWith(
                    waitToSpinUp,
                    waitToAutoAim,
                    waitToFirstIntake
                ),
                firstShoot,
                leftEightIntakeCommand.deadlineWith(intake, tuck),
                leftEightSecondShotCommand.deadlineWith(new SequentialCommandGroup(
                    secondAim,
                    unjam
                )),
                secondShoot,
                readyGyro
            )
        );
    }
    
}

