/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1678.frc2021.auto.AutonomousSelector;
import com.team1678.frc2021.controlboard.ControlBoard;
import com.team1678.frc2021.loops.Looper;
import com.team1678.frc2021.subsystems.Canifier;
import com.team1678.frc2021.subsystems.Climber;
import com.team1678.frc2021.subsystems.Hood;
import com.team1678.frc2021.subsystems.Indexer;
import com.team1678.frc2021.subsystems.Infrastructure;
import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.LEDs;
import com.team1678.frc2021.subsystems.Limelight;
import com.team1678.frc2021.subsystems.RobotStateEstimator;
import com.team1678.frc2021.subsystems.Shooter;
import com.team1678.frc2021.subsystems.Skywalker;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.subsystems.Trigger;
import com.team1678.frc2021.subsystems.Turret;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.wpilib.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

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
    private boolean mPivoted = false;
    private Rotation2d lastTurretJog;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotPeriodic() {		
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
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
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
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		if (m_autonomousCommand != null) {
            Swerve.getInstance().resetOdometry(AutonomousSelector.getStartingPose());
			m_autonomousCommand.schedule();
		}

        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
            mLimelight.setLed(Limelight.LedMode.ON);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mTurret.setNeutralMode(NeutralMode.Brake);
            mHood.setNeutralMode(NeutralMode.Brake);
            mInfrastructure.setIsDuringAuto(true);

            mEnabledLooper.start();

            mTurret.cancelHoming();
            
            mLimelight.setPipeline(Constants.VisionConstants.kPortPipeline);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
        mLimelight.setLed(Limelight.LedMode.ON);


        if (!mLimelight.limelightOK()) {
            mLEDs.conformToState(LEDs.State.EMERGENCY);
        } else if (mSuperstructure.isAimed()) {
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

            mInfrastructure.setIsDuringAuto(false);

            mEnabledLooper.start();
            mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.VisionConstants.kPortPipeline);
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
            Rotation2d turret_jog = mControlBoard.getJogTurret();

            if (!climb_mode) {
                if (!mLimelight.limelightOK()) {
                    mLEDs.conformToState(LEDs.State.EMERGENCY);
                } else if (mSuperstructure.getTucked()) {
                    mLEDs.conformToState(LEDs.State.HOOD_TUCKED);
                } else if (mSuperstructure.isAimed() && mLimelight.seesTarget()) {
                    mLEDs.conformToState(LEDs.State.TARGET_TRACKING);
                } else if (mSuperstructure.isAimed()) {
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

            mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.0));//mControlBoard.getTurretCardinal().rotation);

            if (mControlBoard.climbMode()) {
                climb_mode = true;
                mPivoted = false;
            }

            if (!climb_mode){
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
                    lastTurretJog = turret_jog.rotateBy(Rotation2d.fromDegrees(90.0));
                    
                } else {
                    mSuperstructure.setWantAutoAim(lastTurretJog);
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
                    mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
                } else if (mControlBoard.getTestSpit()) {
                    mSuperstructure.setWantTestSpit();
                } else if (mControlBoard.getRunIntake()) {
                    if (!mSuperstructure.getWantShoot()) {
                        mIntake.setState(Intake.WantedAction.INTAKE);
                    } else {
                        mIntake.setState(Intake.WantedAction.STAY_OUT);
                    }
                } else if (mControlBoard.getRetractIntake()) {
                    mIntake.setState(Intake.WantedAction.RETRACT);
                } else {
                    mIntake.setState(Intake.WantedAction.NONE);
                }
            } else {
                Climber.WantedAction climber_action = Climber.WantedAction.NONE;
                Skywalker.WantedAction skywalker_action = Skywalker.WantedAction.NONE;
                mClimber.setShift(true);
                mSuperstructure.enableIndexer(false);
                mIntake.setState(Intake.WantedAction.NONE);
                mSuperstructure.setWantSpinUp(false);
                mSuperstructure.setWantShoot(false);
                mSuperstructure.setWantPreShot(false);
                mSuperstructure.setWantUnjam(false);

                //Climber control
                if (mControlBoard.getClimberJog() == -1){
                    climber_action = (Climber.WantedAction.JOG_DOWN);
                } else if(mControlBoard.getClimberJog() == 1){
                    climber_action = (Climber.WantedAction.JOG_UP);
                } else if (mControlBoard.getLeaveClimbMode()) {
                    climb_mode = false;
                    mClimber.setShift(false);
                }
                // Skywalker Control
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

            mLEDs.writePeriodicOutputs();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
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

            mInfrastructure.setIsDuringAuto(true);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

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
