/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1678.frc2021.controlboard.ControlBoard;
import com.team1678.frc2021.loops.Looper;

import com.team254.lib.wpilib.TimedRobot;

import java.util.Optional;

import com.team1678.frc2021.auto.AutoModeSelector;
import com.team1678.frc2021.subsystems.*;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import com.team1678.frc2021.auto.AutoModeExecutor;
import com.team1678.frc2021.auto.modes.AutoModeBase;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */

	public static CTREConfigs ctreConfigs;
  
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final Trigger mTrigger = Trigger.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Climber mClimber = Climber.getInstance();
    private final Skywalker mSkywalker = Skywalker.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final Canifier mCanifier = Canifier.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();
    private final Limelight mLimelight = Limelight.getInstance(); 

    // auto instances
    private AutoModeExecutor mAutoModeExecutor;
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

    private boolean climb_mode = false;
    private Rotation2d lastTurretJog = Rotation2d.identity();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    // Called periodically during every robot mode
    @Override
    public void robotPeriodic() {		
        m_sSmartdashInteractions.update();
        RobotState.getInstance().outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();

        SmartDashboard.putBoolean("Climb Mode", climb_mode);
        SmartDashboard.putString("LEDs State", mLEDs.getState().name());
    }

    // Called when the robot starts up, before it connects to FMS and Driver Station
    @Override
    public void robotInit() {

      ctreConfigs = new CTREConfigs();
       m_sSmartdashInteractions = SmartdashInteractions.getInstance();

		try {
			/*
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
            MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", Constants.kCameraStreamPort);
			cameraServer.setSource(camera);
			*/

            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                mSwerve,
                mRobotStateEstimator,
				mCanifier,
				mHood,
                mLimelight, 
                mIntake, 
                mIndexer, 
                mShooter,
                mTrigger,
                mSuperstructure,
                mTurret,
                mInfrastructure,
                mSkywalker,
                mClimber,
                mLEDs
            );

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());

            // mLimelight.setLed(Limelight.LedMode.OFF);

            mSwerve.resetOdometry(new Pose2d());            
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    // Called at the start of autonomous
    @Override
    public void autonomousInit() {
        // System.out.println("Starting auto init: " + Timer.getFPGATimestamp());
		// schedule the autonomous command (example)

        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
        CrashTracker.logAutoInit();

        try {
            
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());

            mTurret.setNeutralMode(NeutralMode.Brake);
            mHood.setNeutralMode(NeutralMode.Brake);
            mInfrastructure.setIsDuringAuto(true);

            mEnabledLooper.start();

            mTurret.cancelHoming();
            
            mLimelight.setPipeline(Constants.VisionConstants.kPortPipeline);

            mAutoModeExecutor.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    // Called periodically during autonomous
    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
        
        mLimelight.setLed(Limelight.LedMode.ON);

        mSwerve.updateSwerveOdometry();

        if (!mLimelight.limelightOK()) {
            mLEDs.conformToState(LEDs.State.EMERGENCY);
        } else if (mSuperstructure.isAimed()) {
            mLEDs.conformToState(LEDs.State.TARGET_TRACKING);
        } else if (mSuperstructure.getLatestAimingParameters().isPresent()) {
            mLEDs.conformToState(LEDs.State.TARGET_VISIBLE);
        } else {
            mLEDs.conformToState(LEDs.State.ENABLED);
        }        
    }

    // Called at the start of teleop
    @Override
    public void teleopInit() {
        // System.out.println("Starting teleop init: " + Timer.getFPGATimestamp());

        try {

            mDisabledLooper.stop();

            mClimber.setBrakeMode(true);
            mInfrastructure.setIsDuringAuto(false);
            mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.VisionConstants.kPortPipeline);
            mTurret.setNeutralMode(NeutralMode.Brake);
            mHood.setNeutralMode(NeutralMode.Brake);
            mLEDs.conformToState(LEDs.State.ENABLED);
            mTurret.cancelHoming();

            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
    
    // Called periodically during teleop (controls go here)
    @Override
    public void teleopPeriodic() {
        try {
            /* SWERVE DRIVE */
            if (mControlBoard.zeroGyro()) {
                mSwerve.zeroGyro();
            }

            mSwerve.updateSwerveOdometry();


            Translation2d swerveTranslation = new Translation2d(mControlBoard.getSwerveTranslation().x(), mControlBoard.getSwerveTranslation().y());
            double swerveRotation = mControlBoard.getSwerveRotation();
            mSwerve.teleopDrive(swerveTranslation, swerveRotation, true, true);

            Rotation2d turret_jog = mControlBoard.getJogTurret();
            mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.0));

            if (mControlBoard.getClimbMode()) {
                climb_mode = true;
            }

            if (!climb_mode){
                mSuperstructure.enableIndexer(true);
                mSuperstructure.setWantUnjam(mControlBoard.getWantUnjam());
                mSuperstructure.setManualZoom(mControlBoard.getManualZoom());
                mSuperstructure.setWantHoodScan(mControlBoard.getWantHoodScan());

                if (mSuperstructure.getWantShoot()) {
                    mControlBoard.setRumble(true);
                } else {
                    mControlBoard.setRumble(false);
                }

                if (turret_jog != null) {
                    mSuperstructure.setWantFieldRelativeTurret(
                       turret_jog.rotateBy(Rotation2d.fromDegrees(90.0)));
                    lastTurretJog = turret_jog.rotateBy(Rotation2d.fromDegrees(90.0));
                } else {
                    mSuperstructure.setWantAutoAim(lastTurretJog);
                }

                if (mControlBoard.getTurretReset()) {
                    mRobotState.resetVision();
                    mRobotState.reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());
                }

                if (mControlBoard.getShoot()) {
                    if (mSuperstructure.isAimed() || mSuperstructure.getWantFendor() || mSuperstructure.getWantSpit() || mSuperstructure.getLatestAimingParameters().isEmpty()) {
                        mSuperstructure.setWantShoot(true);
                    }
                } else if (mControlBoard.getSpinDown()) {
                    mSuperstructure.setWantShoot(false);
                } else if (mControlBoard.getSpinUp()) {
                    mSuperstructure.setWantSpinUp();
                } else if (mControlBoard.getTuck()) {
                    mSuperstructure.setWantTuck(true);
                } else if (mControlBoard.getUntuck()) {
                    mSuperstructure.setWantTuck(false);
                } else if (mControlBoard.getTurretReset()) {
                    mRobotState.resetVision();
                    mRobotState.reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());
                } else if (mControlBoard.getTestSpit()) {
                    mSuperstructure.setWantTestSpit();
                }
                
                if (mControlBoard.getRunIntake()) {
                    if (!mSuperstructure.getWantShoot()) {
                        mIntake.setState(Intake.WantedAction.INTAKE);
                    } else {
                        mIntake.setState(Intake.WantedAction.STAY_OUT);
                    }
                } else if (mControlBoard.getOuttake()) {
                    mIntake.setState(Intake.WantedAction.OUTTAKE);
                } else {
                    mIntake.setState(Intake.WantedAction.NONE);
                }

                if (mControlBoard.getTuck()) {
                    mSuperstructure.setWantTuck(true);
                } else if (mControlBoard.getUntuck()) {
                    mSuperstructure.setWantTuck(false);
                }
            } else {
                Climber.WantedAction climber_action = Climber.WantedAction.NONE;
                Skywalker.WantedAction skywalker_action = Skywalker.WantedAction.NONE;

                mClimber.setShift(true);
                mSuperstructure.enableIndexer(false);
                mIntake.setState(Intake.WantedAction.NONE);
                mSuperstructure.setWantShoot(false);
                mSuperstructure.setWantUnjam(false);

                if (mControlBoard.getClimberJog() == -1){
                    climber_action = (Climber.WantedAction.JOG_DOWN);
                } else if(mControlBoard.getClimberJog() == 1){
                    climber_action = (Climber.WantedAction.JOG_UP);
                } else if (mControlBoard.getLeaveClimbMode()) {
                    climb_mode = false;
                    mClimber.setShift(false);
                }

                switch(mControlBoard.getSkywalker()){
                    case 1:
                        skywalker_action = (Skywalker.WantedAction.SHIFT_LEFT);
                        break;
                    case -1:
                        skywalker_action = (Skywalker.WantedAction.SHIFT_RIGHT);
                        break;
                    case 0:
                        skywalker_action = (Skywalker.WantedAction.NONE);
                        break;
                }

                mLEDs.conformToState(LEDs.State.CLIMB_MODE);

                mClimber.setState(climber_action);
                mSkywalker.setState(skywalker_action);
            }

            if (!climb_mode) {
                if (!mLimelight.limelightOK()) {
                    mLEDs.conformToState(LEDs.State.EMERGENCY);
                } else if (mSuperstructure.getTucked()) {
                    mLEDs.conformToState(LEDs.State.HOOD_TUCKED);
                } else if (mSuperstructure.isAimed() && mLimelight.seesTarget()) {
                    mLEDs.conformToState(LEDs.State.TARGET_TRACKING);
                } else if (mSuperstructure.isAimed()) {
                    mLEDs.conformToState(LEDs.State.INVISIBLE_TARGET_TRACKING);
                } else if (mSuperstructure.getLatestAimingParameters().isPresent() && !mLimelight.seesTarget() && !mSuperstructure.getWantHoodScan()) {
                    mLEDs.conformToState(LEDs.State.TARGET_VISIBLE);
                } else if (mLimelight.seesTarget()) {
                    mLEDs.conformToState(LEDs.State.LIMELIGHT_SEES_ONLY);
                } else {
                    mLEDs.conformToState(LEDs.State.ENABLED);
                }
            }
            mLEDs.writePeriodicOutputs();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    // Called at the start of test
    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");
        try {
            mDisabledLooper.stop();
			mEnabledLooper.stop();			
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    // Called periodically during test mode
    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
    }

    // Called on disable
    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());

            // Reset all auto mode state.

            mDisabledLooper.start();

            mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.triggerOutputs();
            mTurret.setNeutralMode(NeutralMode.Coast);
            mHood.setNeutralMode(NeutralMode.Coast);
            mLimelight.writePeriodicOutputs();
            mLEDs.conformToState(LEDs.State.RAINBOW);

            mDisabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        if (mAutoModeExecutor != null) {
            mAutoModeExecutor.stop();
        }
    
        // Reset all auto mode state.
        mAutoModeSelector.reset();
        mAutoModeSelector.updateModeCreator();
        mAutoModeExecutor = new AutoModeExecutor();    
    }

    // Called periodically when the robot is disabled
    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            mAutoModeSelector.updateModeCreator();
            // ]\[mSwerve.resetAnglesToAbsolute();

            mLimelight.setLed(Limelight.LedMode.OFF);
			mLimelight.writePeriodicOutputs();
			
            if (!mLimelight.limelightOK()) {
                mLEDs.conformToState(LEDs.State.EMERGENCY);
            } else if (mTurret.isHoming() || mHood.isHoming()) {
                mLEDs.conformToState(LEDs.State.RAINBOW);
            } else {
                mLEDs.conformToState(LEDs.State.BREATHING_PINK);
            }

            mLEDs.writePeriodicOutputs();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }

            SmartDashboard.putNumber("Desired Position X", 0.0);
            SmartDashboard.putNumber("Desired Position Y", 0.0);

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
}
