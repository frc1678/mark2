/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021;

import java.util.Arrays;

import com.team1678.frc2021.loops.LimelightProcessor;
import com.team1678.frc2021.loops.LimelightProcessor.Pipeline;
import com.team1678.frc2021.loops.Looper;
import com.team1678.frc2021.loops.QuinticPathTransmitter;
import com.team1678.frc2021.loops.RobotStateEstimator;
import com.team1678.frc2021.subsystems.Pigeon;
import com.team1678.frc2021.subsystems.SubsystemManager;
import com.team1678.frc2021.subsystems.*;
import com.team1678.frc2021.subsystems.Climber;
import com.team1678.frc2021.subsystems.Intake;

import com.team1678.frc2021.subsystems.Swerve;
import com.team1678.frc2021.subsystems.Climber.State;
import com.team1678.frc2021.subsystems.Intake.WantedAction;
import com.team1323.io.SwitchController;
import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.Logger;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

// import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.team254.lib.wpilib.TimedRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	// 364 swerve
	public static CTREConfigs ctreConfigs;
  	private Command m_autonomousCommand;
  	private RobotContainer m_robotContainer;

	private Xbox operator;
	private DriverStation ds;

	private boolean mPivoted = false;
	private boolean climb_mode = false;
	private final boolean oneControllerMode = false;

    // superstructure, manager, and subsystems
	private Superstructure mSuperstructure;
	private SubsystemManager subsystems;
    private Pigeon Pigeon;
	private Climber Climber;
	private Intake mIntake;
	private Canifier mCanifier;
	private LEDs mLEDs;
	private Limelight mLimelight;
	private Turret mTurret;
	private Shooter mShooter;
	private Trigger mTrigger;
	private Climber mClimber;
	private Indexer mIndexer;
    private Hood mHood;

	// private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();

	private RobotState mRobotState = RobotState.getInstance();
	private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
	private final Infrastructure mInfrastructure = Infrastructure.getInstance();

    @Override
    public void robotInit() {
		ctreConfigs = new CTREConfigs();
		m_robotContainer = new RobotContainer();
        
        // instantiate swerve drive
        ds = DriverStation.getInstance();

        // instantiate subsystems\
		mIntake = Intake.getInstance();
		mSuperstructure = Superstructure.getInstance();
		mTurret = Turret.getInstance();
		mShooter = Shooter.getInstance();
		mTrigger = Trigger.getInstance();
		mClimber = Climber.getInstance();
		mIndexer = Indexer.getInstance();
		mCanifier = Canifier.getInstance();
		mHood = Hood.getInstance();	
		mLimelight = Limelight.getInstance();
        
    	subsystems = new SubsystemManager(
			Arrays.asList(/*mLEDs,*/
						  // mRobotStateEstimator,
						  mCanifier,
						  mHood,
						  mLimelight,
						  mIntake,
						  mIndexer,
						  mShooter,
						  mTrigger,
						  mSuperstructure,
						  mTurret,
						  mInfrastructure
						  ));

		Logger.clearLog();
		
		operator = new Xbox(1);

        enabledLooper.register(QuinticPathTransmitter.getInstance());
        enabledLooper.register(LimelightProcessor.getInstance());
        disabledLooper.register(QuinticPathTransmitter.getInstance());
        disabledLooper.register(LimelightProcessor.getInstance());
        subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);
		//CommandScheduler.getInstance().registerSubsystem(swerve);

        // swerve.zeroSensors();
        // swerve.zeroSensors(new Pose2d());
		// swerve.stop();


        // generator.generateTrajectories();

        // qTransmitter.addPaths(auto.getPaths());
        // System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));
    }

    public void allPeriodic() {
		subsystems.outputToSmartDashboard();
		mRobotState.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
		Pigeon.getInstance().outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", ds.isEnabled());
		SmartDashboard.putNumber("Match time", ds.getMatchTime());
	}

	public void autoConfig() {
		// swerve.zeroSensors();
		// swerve.setNominalDriveOutput(0.0);
    }
    
    public void teleopConfig() {
		// swerve.setNominalDriveOutput(0.0);
	}

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
		m_autonomousCommand.schedule();
		}
    }

    @Override
    public void autonomousPeriodic() {
        try {
			allPeriodic();

			if (oneControllerMode){
				oneControllerMode();
			} else {
				twoControllerMode();
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		allPeriodic();
    }

    @Override
    public void testInit() {
		CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        allPeriodic();
    }

    @Override
    public void disabledInit() {
        try {
			enabledLooper.stop();
			subsystems.stop();
			mTurret.setCoastMode();
			mHood.setCoastMode();
			disabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
    }

    @Override
    public void disabledPeriodic() {
        try {
			allPeriodic();
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
			disabledLooper.stop();
			enabledLooper.start();
			teleopConfig();
			SmartDashboard.putBoolean("Auto", false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
    }

    @Override
    public void teleopPeriodic() {
        try {
			operator.update();
			allPeriodic();

			if (oneControllerMode){
				oneControllerMode();
			} else {
				twoControllerMode();
			}
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	public Rotation2d getJogTurret() {
		double jogX = operator.getX(Hand.kLeft);
		double jogY = operator.getY(Hand.kLeft);
		
		Translation2d mag = new Translation2d(jogX, jogY);
		Rotation2d turret = mag.direction();
		return turret;
	}
	
	private void twoControllerMode(){

		Rotation2d turret_jog = getJogTurret();

		if (operator.leftBumper.shortReleased() && operator.rightBumper.shortReleased()  && 
        	operator.leftTrigger.shortReleased() &&  operator.rightTrigger.shortReleased()) {
			climb_mode = true;
			mPivoted = false;
		}
		if (!climb_mode) { //TODO: turret preset stuff and jog turret and rumbles
			
			// if (mIntake.getState() == Intake.State.INTAKING || mIntake.getState() == Intake.State.STAYING_OUT) {
			// 	mSuperstructure.enableIndexer(true);
			// } else {
			// 	mSuperstructure.enableIndexer(false);	
			// }

			
			mSuperstructure.setWantUnjam(operator.leftBumper.shortReleased());
			mSuperstructure.setManualZoom(operator.leftBumper.shortReleased());
			if (mSuperstructure.getWantShoot()) {
				operator.setRumble(RumbleType.kRightRumble, true ? 1 : 0);
			} else {
				operator.setRumble(RumbleType.kRightRumble, false ? 1 : 0);
			}
			mSuperstructure.setWantHoodScan(operator.leftCenterClick.shortReleased());
			if (!turret_jog.equals(Rotation2d.fromDegrees(0))) {
				mSuperstructure.setWantFieldRelativeTurret(
				   turret_jog.rotateBy(Rotation2d.fromDegrees(0)));

			// } else if (false /*return mController.getController().getStickButtonReleased(Hand.kLeft);*/) {
			// 	mSuperstructure.setWantFendor();
			// 	//mSuperstructure.setWantFieldRelativeTurret(Rotation2dd.fromDegrees(180.));
			} else {
				mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(180.0));
				//System.out.println("is doing setWantAutoAim");
			}

			if (operator.getYButtonReleased()) {
				if (mSuperstructure.isAimed() || mSuperstructure.getWantFendor() || mSuperstructure.getWantSpit() || mSuperstructure.getLatestAimingParameters().isEmpty()) {
					mSuperstructure.setWantShoot();
				}
			} else if (operator.getBButton()) {
				mSuperstructure.setWantDisableIndexer();
				// The B button is no longer meant to pre shoot just for disabling the indexer
				//mSuperstructure.setWantPreShot(true);
			} else if (operator.getAButton()) {
				mSuperstructure.setWantSpinUp();
			} else if (operator.getXButton()) {
				mSuperstructure.setWantTuck(true);
			} else if (operator.getStartButton()) {
				mSuperstructure.setWantTuck(false);
			} else if (operator.getBackButtonReleased()) {
				mRobotState.resetVision();
				mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
			} else if (operator.getStickButtonReleased(Hand.kRight)) {
				mSuperstructure.setWantTestSpit();
			} else if (operator.rightTrigger.isBeingPressed()) {
				if (!mSuperstructure.getWantShoot()) {
					mIntake.setState(WantedAction.INTAKE);
				} else {
					mIntake.setState(WantedAction.STAY_OUT);
				}
				mSuperstructure.setAutoIndex(false);
			} else if (operator.leftTrigger.isBeingPressed()) {
				mIntake.setState(WantedAction.RETRACT);
			// } else if (operator.POV0.wasActivated()) {
			// 	mRoller.setState(Roller.WantedAction.ACHIEVE_ROTATION_CONTROL);
			// } else if (operator.POV0.wasActivated()) {
			// 	mRoller.setState(Roller.WantedAction.ACHIEVE_POSITION_CONTROL);
			/*} else if (mControlBoard.getManualFastRoller()) {
				mRoller.runManual(-5.0);
			} else if (mControlBoard.getManualSlowRoller()) {
				mRoller.runManual(-2.0);
			} else if (mControlBoard.getStopManualRoller()) {
				mRoller.stop(); */
			} else {
				mIntake.setState(WantedAction.NONE);
				//mRoller.stop();
			}
		} else {
			if (!operator.POV0.wasActivated()) {
				climb_mode = false;
				//buddy_climb = false;
			}
		}

	}

	private void oneControllerMode(){

	}

}