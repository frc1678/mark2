package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.RobotContainer;
import com.team1678.frc2021.RobotState;
import com.team1678.frc2021.commands.TeleopSwerve;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.lib.math.Conversions;
// import com.team254.lib.geometry.Pose2d;
// import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class RobotStateEstimator extends Subsystem {
    private Swerve mSwerve;
    
	private static RobotStateEstimator mInstance = new RobotStateEstimator();
	private RobotState mRobotState = RobotState.getInstance();
	private RobotContainer m_robotContainer = new RobotContainer();
	private TeleopSwerve mTeleopSwerve;
    private double prev_timestamp_ = -1.0;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
            mSwerve = Swerve.getInstance();
			prev_timestamp_ = timestamp;
		}

        @Override
        public synchronized void onLoop(double timestamp) {
            final double dt = timestamp - prev_timestamp_;

            Pose2d swervePose = mSwerve.getPose();
			double swervePoseTranslation_x = swervePose.getTranslation().getX();
			double swervePoseTranslation_y = swervePose.getTranslation().getY();
			double swervePoseRotation = swervePose.getRotation().getDegrees();
			
			com.team254.lib.geometry.Pose2d mSwervePose = new com.team254.lib.geometry.Pose2d(swervePoseTranslation_x, swervePoseTranslation_y, 
										new com.team254.lib.geometry.Rotation2d(swervePoseRotation));

			mRobotState.addFieldToVehicleObservation(timestamp, mSwervePose.getPose());
			
			mTeleopSwerve = m_robotContainer.getTeleopSwerve();
			Translation2d translation = mTeleopSwerve.getChassisTranslation();
			double rotation = mTeleopSwerve.getChassisRotation();
			ChassisSpeeds chassisVelocity = mSwerve.getChassisVelocity(translation, rotation);

            double vx = Conversions.metersToInches(chassisVelocity.vxMetersPerSecond);
            double vy = Conversions.metersToInches(chassisVelocity.vyMetersPerSecond);
			com.team254.lib.geometry.Twist2d latest_displacement = new com.team254.lib.geometry.Twist2d(vx, vy,
                                Math.toDegrees(chassisVelocity.omegaRadiansPerSecond)).scaled(dt);

			final Twist2d measured_velocity = new Twist2d(vx, vy, chassisVelocity.omegaRadiansPerSecond);
			final Twist2d predicted_velocity = measured_velocity.scaled(dt);
			
			mRobotState.addVehicleToTurretObservation(timestamp,
								new com.team254.lib.geometry.Rotation2d(Turret.getInstance().getAngle()));
			mRobotState.addObservations(timestamp, latest_displacement, measured_velocity, predicted_velocity);
            mRobotState.addVehicleToHoodObservation(timestamp, new com.team254.lib.geometry.Rotation2d(90-Hood.getInstance().getAngle()));
            
        }

        @Override
        public void onStop(double timestamp) {
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mRobotState.outputToSmartDashboard();
    }
}
