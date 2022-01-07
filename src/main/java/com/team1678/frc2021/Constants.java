package com.team1678.frc2021;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.lib.util.SwerveModuleConstants;
import com.team1678.frc2021.subsystems.Limelight.LimelightConstants;
import com.team1678.frc2021.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {
	/*All distance measurements are in inches, unless otherwise noted.*/

	public static final double kLooperDt = 0.02;
	
	public static final double kEpsilon = 0.0001;
	public static double kSwerveRotationEncoderResolution = 4096;
	
	public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kDebuggingOutput = true;

	// Control Board
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

	// Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 36.5;
    public static final double kRobotLength = 36.5;

    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    public static final double kRobotProbeExtrusion = 4.0;
	
	/* 364 IMPORTED CONSTANTS */
	public static final double stickDeadband = 0.15;

	public static final class SwerveConstants {	
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24);
		public static final double wheelBase = Units.inchesToMeters(24);

        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.86;
        public static final double angleGearRatio = 12.8; 

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new edu.wpi.first.wpilibj.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.wpilibj.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Controller Invert */
        public static final boolean invertXAxis = false;
        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final double angleOffset = 144;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER, angleOffset);
        }
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final double angleOffset = 44;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(Ports.FR_DRIVE, Ports.FR_ROTATION, Ports.FR_CANCODER, angleOffset);
        }
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final double angleOffset = 289;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(Ports.BL_DRIVE, Ports.BL_ROTATION, Ports.BL_CANCODER, angleOffset);
        }
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final double angleOffset = 60;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(Ports.BR_DRIVE, Ports.BR_ROTATION, Ports.BR_CANCODER, angleOffset);
        }

	}
	
	public static final class SnapConstants {
        public static final double snapKP = 3.0;
        public static final double snapKI = 0;
        public static final double snapKD = 0.0;
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;

        //Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);
	
		public static final double kSlowMaxSpeedMetersPerSecond = 1.0;
		public static final double kSlowMaxAccelerationMetersPerSecondSquared = 3;

		public static final double kFastMaxSpeedMetersPerSecond = 4;
		public static final double kFastMaxAccelerationMetersPerSecondSquared = 3;
		
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 2.5;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
				
		// Trajectory Speed Configs
		public static final TrajectoryConfig defaultConfig = 
		new TrajectoryConfig(
				Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics);

		public static final TrajectoryConfig slowConfig = 
			new TrajectoryConfig(
				Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics);
		
		public static final TrajectoryConfig fastConfig = 
			new TrajectoryConfig(
				Constants.AutoConstants.kFastMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kFastMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics);
			
		public static final TrajectoryConfig RTNfastConfig = 
			new TrajectoryConfig(
				Constants.AutoConstants.kFastMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kFastMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics);

		public static final TrajectoryConfig RTNFastToZero = 
			new TrajectoryConfig(
				Constants.AutoConstants.kFastMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kFastMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics)
			.setStartVelocity(Constants.AutoConstants.kFastMaxSpeedMetersPerSecond)
			.setEndVelocity(0);
		
		public static final TrajectoryConfig zeroToSlow =
			new TrajectoryConfig(
				Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics)
			.setStartVelocity(0)
			.setEndVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);
			
		public static final TrajectoryConfig intermediateSlow =
			new TrajectoryConfig(
				Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics)
			.setStartVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond)
			.setEndVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);
			


		public static final TrajectoryConfig slowToZero =
			new TrajectoryConfig(
				Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveConstants.swerveKinematics)
    		.setStartVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond)
			.setEndVelocity(0);
			
		public static final TrajectoryConfig zeroToMax =
        	new TrajectoryConfig(
            	Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kSlowMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics)
			.setStartVelocity(0.0)
			.setEndVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond);
			
		    
		public static final TrajectoryConfig maxToZero =
			new TrajectoryConfig(
				Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics)   
			.setStartVelocity(Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond)
			.setEndVelocity(0);
      }

	  public static final class VisionConstants {
		public static final LimelightConstants kLimelightConstants = new LimelightConstants();
		static {
			kLimelightConstants.kName = "Limelight";
			kLimelightConstants.kTableName = "limelight";
			kLimelightConstants.kHeight = 24.5; // inches
			kLimelightConstants.kTurretToLens = Pose2d.identity();
			kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(0.0);
		}

		public static final double kHorizontalFOV = 59.6; // degrees
		public static final double kVerticalFOV = 49.7; // degrees
		public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
		public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
		public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

		public static final double kMaxTrackerDistance = 15.0;
		public static final double kMaxGoalTrackAge = 30.0;
		public static final double kMaxGoalTrackAgeNotTracking = 0.3;
		public static final double kMaxGoalTrackSmoothingTime = 1.5;
		public static final double kTrackStabilityWeight = 0.0;
		public static final double kTrackAgeWeight = 10.0;
		public static final double kTrackSwitchingWeight = 100.0;
		public static final boolean kEnableCachedGoal = true;
		
		public static final double kCameraFrameRate = 90.0;
		public static final double kMinStability = 0.5;
		public static final int kPortPipeline = 0;
		public static final int kBallPipeline = 2;
		public static final double kGoalHeight = 90.0;

		public static final double kInnerGoalDepth = 0;
		public static final Translation2d kVehicleToTurretTranslation = new Translation2d(1.0, 0);
		public static final double kHoodToTurret = 4.25; // center of the turret to the axis of rotation of the hood
		public static final double kLimelightPitchOffset = 17.66; // limelight pitch at hood 0
		public static final double kAutoAimPredictionTime = 0.01; // lookahead for robot state during aiming
	}

	public static final class TurretConstants {
		public static final ServoMotorSubsystemConstants kTurretServoConstants = new ServoMotorSubsystemConstants();
		static {
			kTurretServoConstants.kName = "Turret";
	
			kTurretServoConstants.kMasterConstants.id = Ports.TURRET;
			kTurretServoConstants.kMasterConstants.invert_motor = true;
			kTurretServoConstants.kMasterConstants.invert_sensor_phase = false;
	
			// Unit == Degrees
			kTurretServoConstants.kHomePosition = 0.0; // CCW degrees from forward
			kTurretServoConstants.kTicksPerUnitDistance = (2048.0 * 54.2) / 360.0;
			kTurretServoConstants.kKp = 0.70; // 0.5
			kTurretServoConstants.kKi = 0;
			kTurretServoConstants.kKd = 0.0;
			kTurretServoConstants.kKf = 0.10;
			kTurretServoConstants.kKa = 0.0;
			kTurretServoConstants.kMaxIntegralAccumulator = 0;
			kTurretServoConstants.kIZone = 0; // Ticks
			kTurretServoConstants.kDeadband = 0; // Ticks
	
			kTurretServoConstants.kPositionKp = 0.1;
			kTurretServoConstants.kPositionKi = 0.0;
			kTurretServoConstants.kPositionKd = 10.0;
			kTurretServoConstants.kPositionKf = 0.0;
			kTurretServoConstants.kPositionMaxIntegralAccumulator = 0;
			kTurretServoConstants.kPositionIZone = 0; // Ticks
			kTurretServoConstants.kPositionDeadband = 0; // Ticks
	
			kTurretServoConstants.kMinUnitsLimit = -135.0;
			kTurretServoConstants.kMaxUnitsLimit = 315.0;
	
			kTurretServoConstants.kCruiseVelocity = 20000; // Ticks / 100ms
			kTurretServoConstants.kAcceleration = 40000; // Ticks / 100ms / s
			kTurretServoConstants.kRampRate = 0.0; // s
			kTurretServoConstants.kContinuousCurrentLimit = 20; // amps
			kTurretServoConstants.kPeakCurrentLimit = 40; // amps
			kTurretServoConstants.kPeakCurrentDuration = 10; // milliseconds
			kTurretServoConstants.kMaxVoltage = 12.0;
		}
	
	}

	public static final class HoodConstants {
		public static final ServoMotorSubsystemConstants kHoodServoConstants = new ServoMotorSubsystemConstants();
		static {
			kHoodServoConstants.kName = "Hood";
	
			kHoodServoConstants.kMasterConstants.id = Ports.HOOD;
			kHoodServoConstants.kMasterConstants.invert_motor = true;
			kHoodServoConstants.kMasterConstants.invert_sensor_phase = false;
	
			// Unit == Degrees
			kHoodServoConstants.kHomePosition = 0.0; // Degrees
			kHoodServoConstants.kTicksPerUnitDistance = (2048.0 * 93.8) / 360.0;
			kHoodServoConstants.kKp = 0.55;
			kHoodServoConstants.kKi = 0;
			kHoodServoConstants.kKd = 0;
			kHoodServoConstants.kKf = 0.05;
			kHoodServoConstants.kMaxIntegralAccumulator = 0;
			kHoodServoConstants.kIZone = 0; // Ticks
			kHoodServoConstants.kDeadband = 0; // Ticks
	
			kHoodServoConstants.kPositionKp = 0.1;
			kHoodServoConstants.kPositionKi = 0;
			kHoodServoConstants.kPositionKd = 0;
			kHoodServoConstants.kPositionKf = 0.0;
			kHoodServoConstants.kPositionMaxIntegralAccumulator = 0;
			kHoodServoConstants.kPositionIZone = 0; // Ticks
			kHoodServoConstants.kPositionDeadband = 0; // Ticks
	
			// degrees of limelight pitch from horiz
			kHoodServoConstants.kMinUnitsLimit = 17.66;
			kHoodServoConstants.kMaxUnitsLimit = 89.5;
	
			kHoodServoConstants.kCruiseVelocity = 20000; // Ticks / 100ms
			kHoodServoConstants.kAcceleration = 20000; // Ticks / 100ms / s
			kHoodServoConstants.kRampRate = 0.0; // s
			kHoodServoConstants.kContinuousCurrentLimit = 35; // amps
			kHoodServoConstants.kPeakCurrentLimit = 40; // amps
			kHoodServoConstants.kPeakCurrentDuration = 10; // milliseconds
			kHoodServoConstants.kMaxVoltage = 3.0;
		}
	
		public static final double kHoodRadius = 11.904; // radius of hood
	}

	public static final class IndexerConstants {
		public static final int kSlot0Proxy = 1;
		public static final int kSlot1Proxy = 2;
		public static final int kSlot2Proxy = 3;
		public static final int kSlot3Proxy = 4;
		public static final int kSlot4Proxy = 5;
		public static final int kIndexerLimitSwitch = 6;
	
		public static final double kIndexerKp = 0.2;
		public static final double kIndexerKi = 0.;
		public static final double kIndexerKd = 0.;
		public static final double kIndexerKf = .05;
		public static final double kIndexerVelocityKp = 0.05;
		public static final double kIndexerVelocityKi = 0.;
		public static final double kIndexerVelocityKd = 0.;
		public static final double kIndexerVelocityKf = .05;
		public static final int kIndexerMaxVelocity = 20000; // ticks / 100ms
		public static final int kIndexerMaxAcceleration = 40000; // ticks / 100ms / sec
	
		public static final int kIndexerSlots = 5;
		public static final int kAnglePerSlot = 360 / kIndexerSlots;
		public static final double kIndexerDeadband = 2.0; // degrees
	}

	public static final class ShooterConstants {
		public static final double kShooterP = 0.25;
		public static final double kShooterI = 0.00004;
		public static final double kShooterD = 0.0;
		public static final double kShooterF = 0.05;
	}

	public static final class TriggerConstants {
		public static final double kTriggerP = 0.05;
		public static final double kTriggerI = 0.0;
		public static final double kTriggerD = 0.0;
		public static final double kTriggerF = 0.05;

		public static final double kTriggerRPM = 5000.0;
	}
	

	public static final class SkywalkerConstants {
		public static final double kIdleVoltage = 0.0;
		public static final double kShiftingRightVoltage = 5.0; // 12
		public static final double kShiftingLeftVoltage = -5.0; // -12
	}

	public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId < 8) {
            return new Solenoid(Ports.PCM, solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
	}
  
	public static final int kLongCANTimeoutMs = 100;
	public static final int kCANTimeoutMs = 10;
}
