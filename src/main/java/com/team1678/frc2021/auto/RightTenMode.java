package com.team1678.frc2021.auto;

import com.team1678.frc2021.Constants;

import com.team1678.frc2021.commands.AutoAimCommand;
import com.team1678.frc2021.commands.IntakeCommand;
import com.team1678.frc2021.commands.ShootCommand;
import com.team1678.frc2021.commands.SpinUpCommand;
import com.team1678.frc2021.commands.SwervePointTurnCommand;
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

public class RightTenMode extends SequentialCommandGroup {

    public RightTenMode(Swerve s_Swerve) {
        final Intake mIntake = Intake.getInstance();
        final Superstructure mSuperstructure = Superstructure.getInstance();

        Trajectory firstIntake =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.90, 0.71, Rotation2d.fromDegrees(0.0)),
                List.of(new Translation2d(5.0, 0.71)),
                new Pose2d(6.50, 0.71, Rotation2d.fromDegrees(0.0)),
                Constants.AutoConstants.RTNfastConfig);

        Trajectory closeShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(6.10, 0.71, Rotation2d.fromDegrees(335.0)),
                List.of(new Translation2d(5.10, 0.71),
                        new Translation2d(2.54, 2.54),
                        new Translation2d(1.52, 4.84)
                        ),
                new Pose2d(1.52, 5.84, Rotation2d.fromDegrees(90.0)),
                Constants.AutoConstants.RTNfastConfig);

        Trajectory getFirstShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.9, 0.71, Rotation2d.fromDegrees(0.0)),
                List.of(new Translation2d(6.10, 0.71),
                        new Translation2d(5.10, 0.71),
                        new Translation2d(2.54, 2.54),
                        new Translation2d(1.52, 4.84)
                        ),
                new Pose2d(1.52, 5.84, Rotation2d.fromDegrees(90.0)),
                Constants.AutoConstants.RTNfastConfig);

        Trajectory getToShieldGen =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.52, 5.84, Rotation2d.fromDegrees(330.0)),
                List.of(new Translation2d(3.8, 4.064),
                        new Translation2d(6.096, 7.0)),
                new Pose2d(7.6, 5.49, Rotation2d.fromDegrees(240.0)),
                Constants.AutoConstants.RTNfastConfig);

        Trajectory getToSecondIntake =
             TrajectoryGenerator.generateTrajectory(
                new Pose2d(7.3, 5.49, Rotation2d.fromDegrees(240.0)),
                 List.of(/*new Translation2d(6.1, 4.7)*/),
                 new Pose2d(6.37, 3.72, Rotation2d.fromDegrees(240.0)),
                 Constants.AutoConstants.RTNFastToZero);

        // Trajectory getFirstToSecondIntake =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(7.27, 5.0, Rotation2d.fromDegrees(270.0)),
        //         List.of(),
        //         new Pose2d(6.38, 4.52, Rotation2d.fromDegrees(270.0)),
        //         Constants.AutoConstants.slowToZero);

        // Trajectory getToSecondIntake =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(6.38, 4.52, Rotation2d.fromDegrees(180)),
        //         List.of(),
        //         new Pose2d(5.34, 4.9, Rotation2d.fromDegrees(180)),
        //         Constants.AutoConstants.zeroToSlow);

        // Trajectory getToSecondShot =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(5.34, 4.9, Rotation2d.fromDegrees(180)),
        //         List.of(),
        //         new Pose2d(4.5, 6.0, Rotation2d.fromDegrees(130)),
        //         Constants.AutoConstants.slowToZero);

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
                () -> Rotation2d.fromDegrees(0), //Swerve Heading
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
                () -> Rotation2d.fromDegrees(180), // Swerve Heading
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
                () -> Rotation2d.fromDegrees(250),
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand driveFirstIntakeCommand =
            new SwerveControllerCommand(
                getToSecondIntake,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(50),
                s_Swerve::setModuleStates,
                s_Swerve);

        // SwervePointTurnCommand headingAdjustCommand =
        //     new SwervePointTurnCommand(
        //         s_Swerve::getPose,
        //         Constants.Swerve.swerveKinematics,
        //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //         thetaController,
        //         () -> Rotation2d.fromDegrees(140),
        //         s_Swerve::setModuleStates,
        //         s_Swerve);

        // SwerveControllerCommand driveFirstToSecondIntakeCommand =
        //     new SwerveControllerCommand(
        //         getFirstToSecondIntake,
        //         s_Swerve::getPose,
        //         Constants.Swerve.swerveKinematics,
        //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //         thetaController,
        //         () -> Rotation2d.fromDegrees(270),
        //         s_Swerve::setModuleStates,
        //         s_Swerve);

        // SwerveControllerCommand driveSecondIntakeCommand =
        //     new SwerveControllerCommand(
        //         getToSecondIntake,
        //         s_Swerve::getPose,
        //         Constants.Swerve.swerveKinematics,
        //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //         thetaController,
        //         () -> Rotation2d.fromDegrees(200),
        //         s_Swerve::setModuleStates,
        //         s_Swerve);

        // SwerveControllerCommand driveToSceondShotCommand =
        //     new SwerveControllerCommand(
        //         getToSecondShot,
        //         s_Swerve::getPose,
        //         Constants.Swerve.swerveKinematics,
        //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //         thetaController,
        //         () -> Rotation2d.fromDegrees(200),
        //         s_Swerve::setModuleStates,
        //         s_Swerve);

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

        WaitToSpinUpCommand waitToSpinUp =
            new WaitToSpinUpCommand(mSuperstructure, 0.5);

        WaitToAutoAimCommand waitToAutoAim =
            new WaitToAutoAimCommand(mSuperstructure, 140, 0.5);

        WaitToIntakeCommand waitToFirstIntake =
            new WaitToIntakeCommand(mIntake, mSuperstructure, 0.0);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(2.90, 0.71, Rotation2d.fromDegrees(270.0)))),
            new SequentialCommandGroup(
                firstIntakeCommand.deadlineWith(
                    waitToSpinUp,
                    waitToAutoAim,
                    waitToFirstIntake
                ),
                closeShotCommand
            ),
            new SequentialCommandGroup(
                firstShoot, 
                driveToShieldGenerator.deadlineWith(secondSpinUp),
                driveFirstIntakeCommand,
                endAdjustCommand
                // driveFirstToSecondIntakeCommand,
                // driveSecondIntakeCommand,
                // driveToSceondShotCommand,
                // secondShoot
            ).deadlineWith(intake)
        );
    }
}