package com.team1678.frc2021.subsystems;

import java.util.Optional;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.RobotState;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team1678.frc2021.states.SuperstructureConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
    /* Superstructure Instance */
    private static Superstructure mInstance;
    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    /* Required Subsystem Instances */
    private final Turret mTurret = Turret.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Trigger mTrigger = Trigger.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    public Rotation2d mFieldRelativeTurretGoal = null;

    public enum TurretControlModes {
        FIELD_RELATIVE, VISION_AIMED, OPEN_LOOP, JOGGING
    }

    /* Limelight Aiming Readings */
    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private double mCorrectedRangeToTarget = 0.0;
    private int mTrackId = -1;

    /* Limelight Pipeline Ranges */
    private static final double kZoomedOutRange = 190.0;
    private static final double kZoomedInRange = 220.0;
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;

    /* Superstructure Control Modes */
    private boolean mWantsShoot = false;
    private boolean mWantsSpinUp = false;
    private boolean mWantsTuck = false;
    private boolean mWantsFendor = false;
    private boolean mWantsTestSpit = false;
    private boolean mWantsPreShot = false;
    private boolean mWantsUnjam = false;
    private boolean mWantsHoodScan = false;

    /* Current Status Variables */
    private boolean mGotSpunUp = false;
    private double mCurrentTurret = 0.0;
    private double mCurrentHood = 0.0;

    /* Desired Setpoints */
    private double mTurretSetpoint = 0.0;
    private double mHoodSetpoint = 60.5;
    private double mShooterSetpoint = 4000.0;

    /* Subsystem Control Modes */
    private boolean mEnableIndexer = true;
    private boolean mManualZoom = false;
    private boolean mDisableLimelight = false;
    private TurretControlModes mTurretMode = TurretControlModes.FIELD_RELATIVE;

    /* Enabled Looper */
    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mTurretMode = TurretControlModes.FIELD_RELATIVE;

                    if (SuperstructureConstants.kUseSmartdashboard) {
                        SmartDashboard.putNumber("Shooting RPM", mShooterSetpoint);
                        SmartDashboard.putNumber("Hood Angle", mHoodSetpoint);
                    }                    
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateCurrentState();
                    maybeUpdateGoalFromVision(timestamp);
                    maybeUpdateGoalFromFieldRelativeGoal(timestamp);
                    maybeUpdateGoalFromHoodScan(timestamp);
                    followSetpoint();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    /* Update setpoints from vision if we want to vision aim & have a target */
    public synchronized void maybeUpdateGoalFromVision(double timestamp) {

        /* Exit if we don't want to vision aim */
        if (mTurretMode != TurretControlModes.VISION_AIMED) {
            resetAimingParameters();
            return;
        }

        /* Don't switch targets while shooting */
        if (mWantsShoot && mGotSpunUp) {
            mLatestAimingParameters = mRobotState.getAimingParameters(mTrackId, Constants.VisionConstants.kMaxGoalTrackAge);
        } else {
            mLatestAimingParameters = mRobotState.getAimingParameters(-1, Constants.VisionConstants.kMaxGoalTrackAge);
        }

        /* If we have a target */
        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

            /* Predict robot motion */
            Pose2d robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                    .transformBy(mRobotState.getPredictedFieldToVehicle(Constants.VisionConstants.kAutoAimPredictionTime));
            Pose2d predicted_turret_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getTurretToGoal());
            mCorrectedRangeToTarget = predicted_turret_to_goal.getTranslation().norm();

            /* Don't aim if not in min distance */
            if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance) {
                return;
            }

            /* Update shooting angle and rpm from regression */
            final double shooting_setpoint = getShootingSetpointRpm(mCorrectedRangeToTarget);
            mShooterSetpoint = shooting_setpoint;

            final double aiming_setpoint = getHoodSetpointAngle(mCorrectedRangeToTarget);
            if (!mWantsHoodScan) {
                mHoodSetpoint = aiming_setpoint;
            }

            /* Correct for turret error (with soft limits) */
            final Rotation2d turret_error = mRobotState.getVehicleToTurret(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getTurretToGoalRotation());
            mTurretSetpoint = mCurrentTurret + turret_error.getDegrees();
            safetyReset();


            /* Check if the hood at aiming position */
            final double hood_error = mCurrentHood - mHoodSetpoint;
            if (Util.epsilonEquals(turret_error.getDegrees(), 0.0, 3.0) && Util.epsilonEquals(hood_error, 0.0, 3.0)) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }
        } else {
            mHasTarget = false;
            mOnTarget = false;
        }
    }

    /* Let turret track field relative if we don't have a target */
    public synchronized void maybeUpdateGoalFromFieldRelativeGoal(double timestamp) {
        if (mTurretMode != TurretControlModes.FIELD_RELATIVE && mTurretMode != TurretControlModes.VISION_AIMED) {
            mFieldRelativeTurretGoal = null;
            return;
        }

        /* Let vision control turret if we have a target */
        if (mTurretMode == TurretControlModes.VISION_AIMED && !mLatestAimingParameters.isEmpty()) {
            return;
        }
        if (mFieldRelativeTurretGoal == null) {
            return;
        }

        /* Predict robot rotation */
        final double kLookaheadTime = 4.0;
        Rotation2d turret_error = mRobotState.getPredictedFieldToVehicle(kLookaheadTime)
                .transformBy(mRobotState.getVehicleToTurret(timestamp)).getRotation().inverse()
                .rotateBy(mFieldRelativeTurretGoal);

        /* Set turret setpoint (with soft limits) */
        mTurretSetpoint = mCurrentTurret + turret_error.getDegrees();
        safetyReset();
    }

    /* Jog the hood between upper and lower limits to search for a target */
    public synchronized void maybeUpdateGoalFromHoodScan(double timestamp) {
        if (!mWantsHoodScan) {
            return;
        }
        if (Util.epsilonEquals(mHood.getAngle(), Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit + 10, 10.0)) {
            mHoodSetpoint = Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit - 10;
        } else if (Util.epsilonEquals(mHood.getAngle(), Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit - 10, 10.0)) {
            mHoodSetpoint = Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit + 10;
        }
    }

    /* Ensure turret and hood are within limits */
    public void safetyReset() {
        /* Wrap turret back around if exceding soft limits (disable limelight while resetting) */
        if (mTurretSetpoint < Constants.TurretConstants.kTurretServoConstants.kMinUnitsLimit) {
            mTurretSetpoint += SuperstructureConstants.kTurretDOF;
            Limelight.getInstance().setLed(Limelight.LedMode.OFF);
            mDisableLimelight = true;
        } else if (mTurretSetpoint > Constants.TurretConstants.kTurretServoConstants.kMaxUnitsLimit) {
            mTurretSetpoint -= SuperstructureConstants.kTurretDOF;
            Limelight.getInstance().setLed(Limelight.LedMode.OFF);
            mDisableLimelight = true;
        } else {
            mDisableLimelight = false;
            Limelight.getInstance().setLed(Limelight.LedMode.ON);
        }

        /* Don't allow hood to excede limits */
        if (mHoodSetpoint < Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit) {
            mHoodSetpoint = Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit;
        }
        if (mHoodSetpoint > Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit) {
            mHoodSetpoint = Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit;
        }
    }

    /* Pass setpoints to subsystems */
    public synchronized void followSetpoint() {

        /* Pull values from shuffleboard for tuning */
        if (SuperstructureConstants.kUseSmartdashboard) {
            mShooterSetpoint = getShootingSetpointRpm(0);
            mHoodSetpoint = getHoodSetpointAngle(0);
        }

        /* Set hood position */
        if (mWantsTuck || !mEnableIndexer) {
            mHood.setSetpointPositionPID(Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit, 0);
        } else if (mWantsFendor) {
            mHood.setSetpointMotionMagic(25.0);
            mTurretSetpoint = 180.0;
        } else if (mWantsTestSpit) {
            mHood.setSetpointMotionMagic(Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit);
        } else {
            mHood.setSetpointMotionMagic(mHoodSetpoint);
        }

        /* Subsystem setpoints */
        Indexer.WantedAction indexerAction = Indexer.WantedAction.PASSIVE_INDEX;
        double real_trigger = 0.0;
        double real_shooter = 0.0;
        boolean real_popout = false;

        /* Spin indexer and reverse trigger while intaking */
        if (Intake.getInstance().getState() == Intake.State.INTAKING) {
            indexerAction = Indexer.WantedAction.PASSIVE_INDEX;
            real_trigger = -600.0;
        }

        /* Logic for spinning up and shooting */
        if (mWantsSpinUp) {
            real_shooter = mShooterSetpoint;
            indexerAction = Indexer.WantedAction.PASSIVE_INDEX;
            real_trigger = -600.0;
        } else if (mWantsPreShot) {
            real_shooter = mShooterSetpoint;
            indexerAction = Indexer.WantedAction.HELLA_ZOOM;
            real_trigger = Constants.TriggerConstants.kTriggerRPM;
            real_popout = false;
        } else if (mWantsShoot) {
            real_shooter = mShooterSetpoint;

            /* Add more delay between shots at far ranges to improve accuracy */
            if (mLatestAimingParameters.isPresent()) {
                if (mLatestAimingParameters.get().getRange() > 240.) {
                    indexerAction = Indexer.WantedAction.SLOW_ZOOM;
                } else {
                    indexerAction = Indexer.WantedAction.ZOOM;
                }
            } else {
                indexerAction = Indexer.WantedAction.ZOOM;
            }
            real_trigger = Constants.TriggerConstants.kTriggerRPM;

            /* Activate trigger when shooter and trigger is spun up */
            if (mGotSpunUp) {
                real_popout = true;
            }

            /* Check if trigger and shooter are spun up */
            if (mShooter.spunUp() && mTrigger.spunUp()) {
                mGotSpunUp = true;
            }
        }

        /* Reverse and pop out trigger to clear ballpath */
        if (mWantsUnjam) {
            indexerAction = Indexer.WantedAction.NONE;
            real_popout = true;
            real_trigger = -5000;
        }

        /* Override indexer state */
        if (mEnableIndexer) {
            mIndexer.setState(indexerAction);
        } else {
            mIndexer.setState(Indexer.WantedAction.NONE);
        }

        /* Write trigger goals */
        mTrigger.setPopoutSolenoid(real_popout);
        mTrigger.setVelocity(real_trigger);

        /* Set flywheel setpoints */
        if (Math.abs(real_shooter) < Util.kEpsilon) {
            /* Set flywheel to coast if setpoint is 0, stops hard decel on the shooter wheel */
            mShooter.setOpenLoop(0);
        } else if (mWantsFendor) {
            mShooter.setVelocity(2000);
        } else if (mWantsTestSpit) {
            mShooter.setVelocity(1200);
        } else {
            mShooter.setVelocity(real_shooter);
        }

        /* Set limelight pipelines */
        if (mLatestAimingParameters.isPresent() && !mWantsHoodScan) {
            if (mManualZoom) {
                Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
            } else if (mLatestAimingParameters.get().getRange() > kZoomedInRange
                    && Limelight.getInstance().getPipeline() == Limelight.kDefaultPipeline) {
                Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
            } else if (mLatestAimingParameters.get().getRange() < kZoomedOutRange
                    && Limelight.getInstance().getPipeline() == Limelight.kZoomedInPipeline) {
                Limelight.getInstance().setPipeline(Limelight.kDefaultPipeline);
            }
        } else if (mManualZoom) {
            Limelight.getInstance().setPipeline(Limelight.kZoomedInPipeline);
        } else {
            Limelight.getInstance().setPipeline(Limelight.kDefaultPipeline);
        }

        /* Write turret goals */
        if (mTurretMode == TurretControlModes.OPEN_LOOP || !mEnableIndexer) {
            mTurret.setOpenLoop(0);
        } else {
            mTurret.setSetpointMotionMagic(mTurretSetpoint);
        }
    }


    /* Output to Smartdashboard */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Shooting", mWantsShoot);
        SmartDashboard.putBoolean("Spinning Up", mWantsSpinUp);
        SmartDashboard.putBoolean("System Spun Up", mGotSpunUp);
        SmartDashboard.putBoolean("Pre Shot", mWantsPreShot);

        SmartDashboard.putBoolean("Test Spit", mWantsTestSpit);
        SmartDashboard.putBoolean("Fendor Shot", mWantsFendor);
        SmartDashboard.putBoolean("Tuck", mWantsTuck);

        SmartDashboard.putNumber("Turret Goal", mTurretSetpoint);
        SmartDashboard.putNumber("Hood Goal", mHoodSetpoint);

        SmartDashboard.putBoolean("Is Field Relative", mTurretMode == TurretControlModes.FIELD_RELATIVE);

        SmartDashboard.putBoolean("Has Aiming Parameters", mLatestAimingParameters.isPresent());
        SmartDashboard.putNumber("Distance to Target", mCorrectedRangeToTarget);

        SmartDashboard.putString("Turret Mode", mTurretMode.toString());

        if (SuperstructureConstants.kUseSmartdashboard) {
            SmartDashboard.putNumber("Shooting RPM", mShooterSetpoint);
            SmartDashboard.putNumber("Hood Angle", mHoodSetpoint);
        }

    }

    /* Pull rpm and angle values from regression */
    private double getShootingSetpointRpm(double range) {
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Shooting RPM", 0);
        } else if (SuperstructureConstants.kUseFlywheelAutoAimPolynomial) {
            return SuperstructureConstants.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    private double getHoodSetpointAngle(double range) {
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Hood Angle", 0);
        } else if (SuperstructureConstants.kUseHoodAutoAimPolynomial) {
            return SuperstructureConstants.kHoodAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    /* Reset limelight target */
    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    /* Update current angles */
    public synchronized void updateCurrentState() {
        mCurrentTurret = mTurret.getAngle();
        mCurrentHood = mHood.getAngle();
    }

    /* Inherited Functions */
    @Override
    public void stop() {
        // No-op
    }

    @Override
    public boolean checkSystem() {
        return true;
    }


    /* Setter Functions */
    public synchronized void setGoal(double shooter, double hood, double turret) {
        if ((mTurretMode == TurretControlModes.VISION_AIMED && mHasTarget)) {
            /* Keep setpoints from vision */
        } else {
            mTurretSetpoint = turret;
            mHoodSetpoint = hood;
            mShooterSetpoint = shooter;
        }
    }

    public synchronized void setWantHoodScan(boolean scan) {
        if (scan != mWantsHoodScan) {
            if (scan) {
                mHoodSetpoint = Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit + 10;
            } else {
                mHoodSetpoint = mHood.getAngle();
            }
        }
        mWantsHoodScan = scan;
    }

    public synchronized void setWantFieldRelativeTurret(Rotation2d field_to_turret) {
        mTurretMode = TurretControlModes.FIELD_RELATIVE;
        mFieldRelativeTurretGoal = field_to_turret;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint, boolean enforce_min_distance,
            double min_distance) {
        mTurretMode = TurretControlModes.VISION_AIMED;
        mFieldRelativeTurretGoal = field_to_turret_hint;
        mEnforceAutoAimMinDistance = enforce_min_distance;
        mAutoAimMinDistance = min_distance;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint) {
        setWantAutoAim(field_to_turret_hint, false, 500);
    }

    public synchronized void setTurretOpenLoop(double throttle) {
        mTurretMode = TurretControlModes.OPEN_LOOP;
    }

    public synchronized void jogTurret(double delta) {
        mTurretMode = TurretControlModes.JOGGING;
        mTurretSetpoint += delta;
    }

    public synchronized void setWantShoot() {
        mWantsSpinUp = false;
        mWantsShoot = !mWantsShoot;
        mGotSpunUp = false;
        mWantsPreShot = false;
    }
    
    public synchronized void setWantShoot(boolean shoot) {
        mWantsSpinUp = false;
        mWantsShoot = shoot;
        mGotSpunUp = false;
        mWantsPreShot = false;
    }

    public synchronized void setWantPreShot(boolean pre_shot) {
        mWantsSpinUp = false;
        mWantsShoot = false;
        mGotSpunUp = false;
        mWantsPreShot = pre_shot;
    }

    public synchronized void setWantSpinUp() {
        mWantsSpinUp = !mWantsSpinUp;
        mWantsShoot = false;
        mGotSpunUp = false;
        mWantsPreShot = false;
    }

    public synchronized void setWantSpinUp(boolean spin_up) {
        mWantsSpinUp = spin_up;
        mWantsShoot = false;
        mWantsPreShot = false;
    }

    public synchronized void setManualZoom(boolean zoom) {
        mManualZoom = zoom;
    }

    public synchronized void setWantTuck(boolean tuck) {
        mWantsTuck = tuck;
    }

    public synchronized void setWantFendor() {
        mWantsFendor = !mWantsFendor;
        if (mWantsFendor) {
            mRobotState.resetVision();
        }
    }
    
    public synchronized void setWantTestSpit() {
        mWantsTestSpit = !mWantsTestSpit;
    }

    public synchronized void setWantUnjam(boolean unjam) {
        mWantsUnjam = unjam;
    }

    public synchronized void enableIndexer(boolean indexer) {
        mEnableIndexer = indexer;
    }

    /* Setter Functions */
    public synchronized boolean spunUp() {
        return mGotSpunUp;
    }

    public boolean getWantShoot() {
        return mWantsShoot;
    }

    public synchronized boolean getWantsShoot() {
        return mWantsShoot;
    }

    public synchronized boolean isAimed() {
        return mOnTarget;
    }

    public synchronized double getCorrectedRangeToTarget() {
        return mCorrectedRangeToTarget;
    }

    public synchronized TurretControlModes getTurretControlMode() {
        return mTurretMode;
    }

    public synchronized boolean getWantHoodScan() {
        return mWantsHoodScan;
    }

    public synchronized boolean getLimelightDisabled() {
        return mDisableLimelight;
    }

    public boolean isAutoAiming(){
        return mTurretMode == TurretControlModes.VISION_AIMED;
    }
 
    public synchronized Optional<AimingParameters> getLatestAimingParameters() {
        return mLatestAimingParameters;
    }

    public synchronized boolean getWantFendor() {
        return mWantsFendor;
    }

    public synchronized boolean getWantSpit() {
        return mWantsTestSpit;
    }

    public synchronized boolean getTucked() {
        return mWantsTuck;
    }

}
