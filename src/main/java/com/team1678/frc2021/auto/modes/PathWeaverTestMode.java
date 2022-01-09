package com.team1678.frc2021.auto.modes;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.auto.AutoModeEndedException;
import com.team1678.frc2021.auto.WaypointReader;
import com.team1678.frc2021.auto.actions.LambdaAction;
import com.team1678.frc2021.auto.actions.SwerveTrajectoryAction;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class PathWeaverTestMode extends AutoModeBase {
    // Swerve instance 
    private final Swerve s_Swerve = Swerve.getInstance();

    // required PathWeaver trajectory paths
    String path = "paths/s-curve.path";

    // trajectories
    Trajectory samplePathWeaverTrajectory;
    SwerveTrajectoryAction samplePathWeaverTrajectoryAction;

    public PathWeaverTestMode() {

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            TrajectoryGenerator.ControlVectorList control_vectors = WaypointReader.getControlVectors(trajectoryPath);
            samplePathWeaverTrajectory = TrajectoryGenerator.generateTrajectory(control_vectors, Constants.AutoConstants.defaultConfig);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }
        
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        samplePathWeaverTrajectoryAction = new SwerveTrajectoryAction(samplePathWeaverTrajectory,
            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
            s_Swerve::setModuleStates);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running test mode auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> s_Swerve.resetOdometry(samplePathWeaverTrajectoryAction.getInitialPose())));

        runAction(samplePathWeaverTrajectoryAction);
        
        System.out.println("Finished auto!");
    }
}
