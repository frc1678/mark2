package com.team1678.frc2021.auto.modes;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.auto.AutoModeEndedException;
import com.team1678.frc2021.auto.actions.LambdaAction;
import com.team1678.frc2021.auto.actions.ParallelAction;
import com.team1678.frc2021.auto.actions.SeriesAction;
import com.team1678.frc2021.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2021.auto.actions.WaitAction;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class FigureEightTestMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve s_Swerve = Swerve.getInstance();

    // trajectory actions
    SwerveTrajectoryAction firstAction;
    SwerveTrajectoryAction secondAction;
    SwerveTrajectoryAction thirdAction;
    SwerveTrajectoryAction fourthAction;
    SwerveTrajectoryAction fifthAction;
    SwerveTrajectoryAction sixthAction;
    SwerveTrajectoryAction seventhAction;
    SwerveTrajectoryAction eigthAction;

    SwerveTrajectoryAction figureEightAction;

    public FigureEightTestMode() {
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveConstants.swerveKinematics);

        Trajectory figureEight = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(Math.PI/2)),
                List.of(new Translation2d(0.114, 0.574),
                        new Translation2d(0.439, 1.061),
                        new Translation2d(0.926, 1.386),
                        new Translation2d(1.5, 1.5),
                        new Translation2d(2.074, 1.386),
                        new Translation2d(2.561, 1.061),
                        new Translation2d(2.886, 0.574),
                        new Translation2d(3, 0),
                        new Translation2d(3.114, -0.574),
                        new Translation2d(3.439, -1.061),
                        new Translation2d(3.926, -1.386),
                        new Translation2d(4.5, -1.5),
                        new Translation2d(5.074, -1.386),
                        new Translation2d(5.561, -1.061),
                        new Translation2d(5.886, -0.574),
                        new Translation2d(6, 0),
                        new Translation2d(5.886, 0.574),
                        new Translation2d(5.561, 1.061),
                        new Translation2d(5.074, 1.386),
                        new Translation2d(4.5, 1.5),
                        new Translation2d(3.926, 1.386),
                        new Translation2d(3.439, 1.061),
                        new Translation2d(3.114, 0.574),
                        new Translation2d(3, 0),
                        new Translation2d(2.886, -0.574),
                        new Translation2d(2.561, -1.061),
                        new Translation2d(2.074, -1.386),
                        new Translation2d(1.5, -1.5),
                        new Translation2d(0.926, -1.386),
                        new Translation2d(0.439, -1.061),
                        new Translation2d(0.114, -0.574)),
                new Pose2d(0, 0, new Rotation2d(Math.PI/2)), Constants.AutoConstants.defaultConfig);


        Trajectory first = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(90)),
                List.of(new Translation2d(0.114, 0.574),
                        new Translation2d(0.439, 1.061),
                        new Translation2d(0.926, 1.386)),
                new Pose2d(1.5, 1.5, new Rotation2d(0)), Constants.AutoConstants.zeroToSlow);

        Trajectory second = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.5, 1.5, new Rotation2d(0)),
                List.of(new Translation2d(2.074, 1.386),
                        new Translation2d(2.561, 1.061),
                        new Translation2d(2.886, 0.574)),
                new Pose2d(3, 0, new Rotation2d(-90)), Constants.AutoConstants.intermediateSlow);

        Trajectory third = TrajectoryGenerator.generateTrajectory(
                new Pose2d(3, 0, new Rotation2d(-90)),
                List.of(new Translation2d(3.114, -0.574),
                        new Translation2d(3.439, -1.061),
                        new Translation2d(3.926, -1.386)),
                new Pose2d(4.5, -1.5, new Rotation2d(0)), Constants.AutoConstants.intermediateSlow);

        Trajectory fourth = TrajectoryGenerator.generateTrajectory(
                new Pose2d(4.5, -1.5, new Rotation2d(0)),
                List.of(new Translation2d(5.074, -1.386),
                        new Translation2d(5.561, -1.061),
                        new Translation2d(5.886, -0.574)),
                new Pose2d(6, 0, new Rotation2d(90)), Constants.AutoConstants.intermediateSlow);

        Trajectory fifth = TrajectoryGenerator.generateTrajectory(
                new Pose2d(6, 0, new Rotation2d(90)),
                List.of(new Translation2d(5.886, 0.574),
                        new Translation2d(5.561, 1.061),
                        new Translation2d(5.074, 1.386)),
                new Pose2d(4.5, 1.5, new Rotation2d(179)), Constants.AutoConstants.intermediateSlow);

        Trajectory sixth = TrajectoryGenerator.generateTrajectory(
                new Pose2d(4.5, 1.5, new Rotation2d(179)),
                List.of(new Translation2d(3.926, 1.386),
                        new Translation2d(3.439, 1.061),
                        new Translation2d(3.114, 0.574)),
                new Pose2d(3, 0, new Rotation2d(269)), Constants.AutoConstants.intermediateSlow);

        Trajectory seventh = TrajectoryGenerator.generateTrajectory(
                new Pose2d(3, 0, new Rotation2d(269)),
                List.of(new Translation2d(2.886, -0.574),
                        new Translation2d(2.561, -1.061),
                        new Translation2d(2.074, -1.386)),
                new Pose2d(1.5, -1.5, new Rotation2d(181)), Constants.AutoConstants.intermediateSlow);

        Trajectory eigth = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.5, -1.5, new Rotation2d(181)),
                List.of(new Translation2d(0.926, -1.386),
                        new Translation2d(0.439, -1.061),
                        new Translation2d(0.114, -0.574)),
                new Pose2d(0, 0, new Rotation2d(91)), Constants.AutoConstants.slowToZero);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        firstAction = new SwerveTrajectoryAction(first,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0.0),
                s_Swerve::setModuleStates);

        secondAction = new SwerveTrajectoryAction(second,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0.0),
                s_Swerve::setModuleStates);

        thirdAction = new SwerveTrajectoryAction(third,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0.0),
                s_Swerve::setModuleStates);

        fourthAction = new SwerveTrajectoryAction(fourth,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0.0),
                s_Swerve::setModuleStates);

        fifthAction = new SwerveTrajectoryAction(fifth,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0.0),
                s_Swerve::setModuleStates);

        sixthAction = new SwerveTrajectoryAction(sixth,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0.0),
                s_Swerve::setModuleStates);

        seventhAction = new SwerveTrajectoryAction(seventh,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0.0),
                s_Swerve::setModuleStates);

        eigthAction = new SwerveTrajectoryAction(eigth,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0.0),
                s_Swerve::setModuleStates);

        figureEightAction = new SwerveTrajectoryAction(figureEight,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(180.0),
                s_Swerve::setModuleStates);
                
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running test mode auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> s_Swerve.resetOdometry(new Pose2d(0,0, new Rotation2d()))));

        // runAction(firstAction);
        // runAction(secondAction);
        // runAction(thirdAction);
        // runAction(fourthAction);
        // runAction(fifthAction);
        // runAction(sixthAction);
        // runAction(seventhAction);
        // runAction(eigthAction);

        runAction(figureEightAction);
        
        System.out.println("Finished auto!");
    }
}
