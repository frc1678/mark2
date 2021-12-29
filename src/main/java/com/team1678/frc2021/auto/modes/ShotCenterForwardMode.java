package com.team1678.frc2021.auto.modes;

import java.util.List;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.auto.AutoModeEndedException;
import com.team1678.frc2021.auto.actions.LambdaAction;
import com.team1678.frc2021.auto.actions.ReadyGyroAction;
import com.team1678.frc2021.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2021.auto.actions.WaitAction;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class ShotCenterForwardMode extends AutoModeBase {
    // Swerve instance 
    private final Swerve s_Swerve = Swerve.getInstance();

    // trajectory actions
    SwerveTrajectoryAction moveForwardAction;

    public ShotCenterForwardMode() {

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory moveFront = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.90, 5.84, Rotation2d.fromDegrees(0.0)), List.of(),
                new Pose2d(3.90, 5.84, Rotation2d.fromDegrees(0.0)), Constants.AutoConstants.defaultConfig);

        moveForwardAction = new SwerveTrajectoryAction(moveFront, s_Swerve::getPose,
                Constants.SwerveConstants.swerveKinematics, new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                () -> Rotation2d.fromDegrees(0), s_Swerve::setModuleStates);
                
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running center front auto mode!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> s_Swerve.resetOdometry(moveForwardAction.getInitialPose())));

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(com.team254.lib.geometry.Rotation2d.fromDegrees(180.0))));
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitAction(5.0));
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));
        runAction(moveForwardAction);
        runAction(new ReadyGyroAction(s_Swerve));

        System.out.println("Finished driving");

    }
    
}