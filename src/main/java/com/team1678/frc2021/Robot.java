/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021;

import java.util.Optional;

import com.team1678.frc2021.controlboard.ControlBoard;
import com.team1678.frc2021.loops.Looper;
import com.team1678.frc2021.controlboard.ControlBoard;
import com.team1678.frc2021.controlboard.GamepadButtonControlBoard;

import com.team254.lib.wpilib.TimedRobot;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.util.*;
import com.team254.lib.wpilib.TimedRobot;

import java.util.Optional;

import com.team1678.frc2021.SubsystemManager;
import com.team1678.frc2021.subsystems.*;
import com.team254.lib.util.*;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.geometry.Rotation2d;
import com.team1678.frc2021.subsystems.Indexer.WantedAction;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.wpilib.TimedRobot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
  	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();

    private final Intake mIntake = Intake.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Trigger mTrigger = Trigger.getInstance();
    private final Climber mClimber = Climber.getInstance();
    private final Skywalker mSkywalker = Skywalker.getInstance();
    private final Hood mHood = Hood.getInstance();

    private final Canifier mCanifier = Canifier.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private boolean climb_mode = false;
    private boolean buddy_climb = false;
    private boolean mPivoted = false;

    // private LoggingSystem mLogger = LoggingSystem.getInstance();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		
        RobotState.getInstance().outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();

        SmartDashboard.putBoolean("Climb Mode", climb_mode);
        SmartDashboard.putBoolean("Pivoted", mPivoted);
        SmartDashboard.putString("LEDs State", mLEDs.getState().name());
    }

    @Override
    public void robotInit() {

		ctreConfigs = new CTREConfigs();
    	// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    	// autonomous chooser on the dashboard.
 	   	m_robotContainer = new RobotContainer();
		
		try {
			/*
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
            MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", Constants.kCameraStreamPort);
			cameraServer.setSource(camera);
			*/

            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                // mRobotStateEstimator,
				// mCanifier,
				// mHood,
                // mLimelight, 
                // mIntake, 
                // mIndexer, 
                // mShooter,
                // mTrigger,
                // mSuperstructure,
                // mTurret,
                // mInfrastructure,
                mSkywalker//,
                // mClimber,
                // mLEDs
            );

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());

			mLimelight.setLed(Limelight.LedMode.OFF);
			
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}

        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
            mLimelight.setLed(Limelight.LedMode.ON);

            mLimelight.setPipeline(Constants.kPortPipeline);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mTurret.setNeutralMode(NeutralMode.Brake);
            mHood.setNeutralMode(NeutralMode.Brake);
            mInfrastructure.setIsDuringAuto(true);

            mEnabledLooper.start();

            mTurret.cancelHoming();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
        //mLimelight.setLed(Limelight.LedMode.ON);

        if (!mLimelight.limelightOK()) {
            mLEDs.conformToState(LEDs.State.EMERGENCY);
        } else if (mSuperstructure.isOnTarget()) {
            mLEDs.conformToState(LEDs.State.TARGET_TRACKING);
        } else if (mSuperstructure.getLatestAimingParameters().isPresent()) {
            mLEDs.conformToState(LEDs.State.TARGET_VISIBLE);
        } else {
            mLEDs.conformToState(LEDs.State.ENABLED);
        }

        try {

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            mClimber.setBrakeMode(true);
            mClimber.setShift(false);

            mInfrastructure.setIsDuringAuto(false);

            //mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();
            mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.kPortPipeline);
            mTurret.setNeutralMode(NeutralMode.Brake);
            mHood.setNeutralMode(NeutralMode.Brake);
            mLEDs.conformToState(LEDs.State.ENABLED);
            mTurret.cancelHoming();
            
            mControlBoard.reset();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
    
    @Override
    public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
            double hood_jog = mControlBoard.getJogHood();
            Rotation2d turret_jog = mControlBoard.getJogTurret();

            if (!climb_mode) {
                if (!mLimelight.limelightOK()) {
                    mLEDs.conformToState(LEDs.State.EMERGENCY);
                } else if (mSuperstructure.getTucked()) {
                    mLEDs.conformToState(LEDs.State.HOOD_TUCKED);
                } else if (mSuperstructure.isOnTarget() && mLimelight.seesTarget()) {
                    mLEDs.conformToState(LEDs.State.TARGET_TRACKING);
                } else if (mSuperstructure.isOnTarget()) {
                    mLEDs.conformToState(LEDs.State.INVISIBLE_TARGET_TRACKING);
                } else if (mSuperstructure.getLatestAimingParameters().isPresent() && !mLimelight.seesTarget() && !mSuperstructure.getScanningHood()) {
                    mLEDs.conformToState(LEDs.State.TARGET_VISIBLE);
                } else if (mLimelight.seesTarget()) {
                    mLEDs.conformToState(LEDs.State.LIMELIGHT_SEES_ONLY);
                } else {
                    mLEDs.conformToState(LEDs.State.ENABLED);
                }
            }

            if (mControlBoard.getShotUp()) {
                mSuperstructure.setAngleAdd(1.0);
            } else if (mControlBoard.getShotDown()) {
                mSuperstructure.setAngleAdd(-1.0);
            }

            //mLimelight.setLed(Limelight.LedMode.ON);        
            
            mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.0));//mControlBoard.getTurretCardinal().rotation);

            if (mControlBoard.climbMode()) {
                climb_mode = true;
                mPivoted = false;
            }

            if (!climb_mode){ //TODO: turret preset stuff and jog turret and rumbles
                mSuperstructure.enableIndexer(true);
                mSuperstructure.setWantUnjam(mControlBoard.getWantUnjam());
                mSuperstructure.setManualZoom(mControlBoard.getManualZoom());

                if (mSuperstructure.getWantShoot()) {
                    mControlBoard.setRumble(true);
                } else {
                    mControlBoard.setRumble(false);
                }

                mSuperstructure.setWantHoodScan(mControlBoard.getWantHoodScan());

                if (turret_jog != null) {
                    mSuperstructure.setWantFieldRelativeTurret(
                       turret_jog.rotateBy(Rotation2d.fromDegrees(90.0)));
                } else if (mControlBoard.getFendorShot()) {
                    mSuperstructure.setWantFendor();
                    //mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.));
                } else {
                    mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(180.0));
                }

                if (mControlBoard.getShoot()) {
                    if (mSuperstructure.isAimed() || mSuperstructure.getWantFendor() || mSuperstructure.getWantSpit() || mSuperstructure.getLatestAimingParameters().isEmpty()) {
                        mSuperstructure.setWantShoot();
                    }
                } else if (mControlBoard.getPreShot()) {
                    mSuperstructure.setWantPreShot(true);
                } else if (mControlBoard.getSpinUp()) {
                    mSuperstructure.setWantSpinUp();
                } else if (mControlBoard.getTuck()) {
                    mSuperstructure.setWantTuck(true);
                } else if (mControlBoard.getUntuck()) {
                    mSuperstructure.setWantTuck(false);
                } else if (mControlBoard.getTurretReset()) {
                    mRobotState.resetVision();
                    mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
                } else if (mControlBoard.getTestSpit()) {
                    mSuperstructure.setWantTestSpit();
                } else if (mControlBoard.getRunIntake()) {
                    if (!mSuperstructure.getWantShoot()) {
                        mIntake.setState(Intake.WantedAction.INTAKE);
                    } else {
                        mIntake.setState(Intake.WantedAction.STAY_OUT);
                    }
                    mSuperstructure.setAutoIndex(false);
                } else if (mControlBoard.getRetractIntake()) {
                    mIntake.setState(Intake.WantedAction.RETRACT);
                } else {
                    mIntake.setState(Intake.WantedAction.NONE);
                    //mRoller.stop();
                }
            } else {
                Climber.WantedAction climber_action = Climber.WantedAction.NONE;
                mSuperstructure.enableIndexer(false);
                mIntake.setState(Intake.WantedAction.NONE);
                mSuperstructure.setWantSpinUp(false);
                mSuperstructure.setWantShoot(false);
                mSuperstructure.setWantPreShot(false);
                mSuperstructure.setWantUnjam(false);

                mClimber.setShift(true);

                //Climber control
                if (mControlBoard.getArmExtend()) { // Press A
                        climber_action = (Climber.WantedAction.EXTEND);   
				} else if (mControlBoard.getArmHug()) { // Press B
                    climber_action = (Climber.WantedAction.HUG); // hook onto the rung
                } else if (mControlBoard.getClimb()) { // Press Y
                    climber_action = (Climber.WantedAction.CLIMB);
                } else if (mControlBoard.getBrake()) { // Release Y
                    climber_action = (Climber.WantedAction.BRAKE);
                } else if (mControlBoard.getLeaveClimbMode()) {
                    climb_mode = false;
                    buddy_climb = false;
                } else {
					// TODO: Check if NONE state needs to be set
                }
                //Skywalker Control
                switch(mControlBoard.getSkywalker()){
                    case -1:
                        mSkywalker.setState(com.team1678.frc2021.subsystems.Skywalker.WantedAction.SHIFT_LEFT);
                        break;
                    case 1:
                        mSkywalker.setState(com.team1678.frc2021.subsystems.Skywalker.WantedAction.SHIFT_RIGHT);
                        break;
                    case 0:
                        mSkywalker.setState(com.team1678.frc2021.subsystems.Skywalker.WantedAction.NONE);
                        break;
                }
                if (mClimber.getState() == Climber.State.HUGGING) {
                    mLEDs.conformToState(buddy_climb ? LEDs.State.HUGGING_BUDDY : LEDs.State.HUGGING);
                } else if (mClimber.getState() == Climber.State.CLIMBING) {
                    mLEDs.conformToState(buddy_climb ? LEDs.State.CLIMBING_BUDDY : LEDs.State.CLIMBING);
                } else {
                    mLEDs.conformToState(buddy_climb ? LEDs.State.CLIMB_MODE_BUDDY : LEDs.State.CLIMB_MODE);
                }

//                if (mControlBoard.getStopExtend() || mControlBoard.getStopClimb()) {
//                    climber_action = Climber.WantedAction.STOP;
//                }

                mClimber.setState(climber_action);
            }
            mLEDs.writePeriodicOutputs();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
		CommandScheduler.getInstance().cancelAll();

        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
			mEnabledLooper.stop();
			
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            mClimber.setBrakeMode(true);

          //  mRobotState.resetVision();

            mInfrastructure.setIsDuringAuto(true);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            // Reset all auto mode state.

            mDisabledLooper.start();

            mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.triggerOutputs();

            mTurret.setNeutralMode(NeutralMode.Coast);
            mHood.setNeutralMode(NeutralMode.Coast);
            mLimelight.writePeriodicOutputs();
            mLEDs.conformToState(LEDs.State.RAINBOW);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        // mLimelight.setStream(2);

        try {
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

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
}
