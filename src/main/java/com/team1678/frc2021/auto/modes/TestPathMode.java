package com.team1678.frc2021.auto.modes;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.auto.AutoModeEndedException;
import com.team1678.frc2021.auto.WaypointReader;
import com.team1678.frc2021.auto.actions.LambdaAction;
import com.team1678.frc2021.auto.actions.ParallelAction;
import com.team1678.frc2021.auto.actions.SeriesAction;
import com.team1678.frc2021.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2021.auto.actions.WaitAction;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class TestPathMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve s_Swerve = Swerve.getInstance();

    // required PathWeaver trajectory paths
    String path_one = "paths/test-part-1.path";
    String path_two = "paths/test-part-2.path";

	// trajectories
	Trajectory straightTrajectory;
	SwerveTrajectoryAction straightTrajectoryAction;
	
	Trajectory rotatingTrajectory;
    SwerveTrajectoryAction rotatingTrajectoryAction;

    public TestPathMode() {
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveConstants.swerveKinematics);

        try {

          Path traj_path_one = Filesystem.getDeployDirectory().toPath().resolve(path_one);
          TrajectoryGenerator.ControlVectorList cv_one = WaypointReader.getControlVectors(traj_path_one);
		  straightTrajectory = TrajectoryGenerator.generateTrajectory(cv_one, Constants.AutoConstants.zeroToSlow);
		  
		  Path traj_path_two = Filesystem.getDeployDirectory().toPath().resolve(path_two);
          TrajectoryGenerator.ControlVectorList cv_two = WaypointReader.getControlVectors(traj_path_two);
          rotatingTrajectory = TrajectoryGenerator.generateTrajectory(cv_two, Constants.AutoConstants.slowToZero);

        } catch (IOException ex) {
		  DriverStation.reportError("Unable to open trajectory: " + path_one, ex.getStackTrace());
		  DriverStation.reportError("Unable to open trajectory: " + path_two, ex.getStackTrace());
        }
        /*
        Trajectory straightTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1.5, 0)),
                new Pose2d(3, 0, new Rotation2d(0)), Constants.AutoConstants.zeroToSlow);
        
        Trajectory rotatingTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(3, 0, new Rotation2d(0)),
                List.of(new Translation2d(4.5, 0)),
                new Pose2d(6, 0, new Rotation2d(0)), Constants.AutoConstants.slowToZero);
        */

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        straightTrajectoryAction = new SwerveTrajectoryAction(straightTrajectory,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                s_Swerve::setModuleStates);
                
        rotatingTrajectoryAction = new SwerveTrajectoryAction(rotatingTrajectory,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                () -> Rotation2d.fromDegrees(179.0),
                s_Swerve::setModuleStates);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running test mode auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> s_Swerve.resetOdometry(straightTrajectoryAction.getInitialPose())));

        // runAction(straightTrajectoryAction);
        // runAction(rotatingTrajectoryAction);

        runAction(
                new ParallelAction(Arrays.asList(
                        straightTrajectoryAction,
                        new SeriesAction(Arrays.asList(
                                new WaitAction(.5),
                                new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE))
                        ))
                ))
        );

        runAction(
                new ParallelAction(Arrays.asList(
                        rotatingTrajectoryAction,
                        new SeriesAction(Arrays.asList(
                                new WaitAction(.5),
                                new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE))
                        ))
                ))       
        );
        
        System.out.println("Finished auto!");
    }
}
