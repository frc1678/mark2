package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
    private static Indexer mInstance = null;

    private static final double kZoomingVelocity = 80.;
    private static final double kPassiveIndexingVelocity = 80.0;
    private static final double kGearRatio = (60. / 16.) * (154. / 16.);

    private static final double kJamCurrent = 150.0;
    private double mLastCurrentSpikeTime = 0.0;
    private static final double kCurrentIgnoreTime = 1.0; 
    
    public static class PeriodicIO {
        // INPUTS
        public double timestamp;

        public double indexer_angle;
        public double indexer_velocity;
        public double indexer_current;
        public boolean snapped;

        // OUTPUTS
        public ControlMode indexer_control_mode = ControlMode.PercentOutput;
        public double indexer_demand;
    }

    public enum WantedAction {
        NONE, PASSIVE_INDEX, ZOOM, SLOW_ZOOM, HELLA_ZOOM,
    }

    public enum State {
        IDLE, PASSIVE_INDEXING,  ZOOMING, SLOW_ZOOMING, HELLA_ZOOMING,
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonFX mMaster;
    private State mState = State.IDLE;
    private boolean mBackwards = false;
    private double mOffset = 0;

    private Indexer() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);
        mMaster.changeMotionControlFramePeriod(255);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 125);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 125);

        mMaster.config_kP(0, Constants.kIndexerKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kIndexerKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kIndexerKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kIndexerKf, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(1, Constants.kIndexerVelocityKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(1, Constants.kIndexerVelocityKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(1, Constants.kIndexerVelocityKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(1, Constants.kIndexerVelocityKf, Constants.kLongCANTimeoutMs);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(Constants.kIndexerMaxVelocity);
        mMaster.configMotionAcceleration(Constants.kIndexerMaxAcceleration);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.0);
    }
    
    public synchronized State getState() {
        return mState;
    }

    public synchronized static Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("IndexerControlMode", mPeriodicIO.indexer_control_mode.name());
        SmartDashboard.putNumber("IndexerSetpoint", mPeriodicIO.indexer_demand);
        SmartDashboard.putNumber("IndexerAngle", mPeriodicIO.indexer_angle);
        SmartDashboard.putNumber("IndexerVelocity", mPeriodicIO.indexer_velocity);
        SmartDashboard.putNumber("IndexerOffset", mOffset);
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.indexer_control_mode = ControlMode.PercentOutput;
        mPeriodicIO.indexer_demand = percentage;
    }

    @Override
    public void stop() {
        setOpenLoop(0);
        mMaster.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                mMaster.setNeutralMode(NeutralMode.Coast);
            }
        });
    }

    public synchronized double getIndexerTheta() {
        return mPeriodicIO.indexer_angle;
    }

    public synchronized void setBackwardsMode(boolean backwards) {
        mBackwards = backwards;
    }
    
    public void runStateMachine() {
        switch (mState) {
        case IDLE:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = 0;
            break;
        case PASSIVE_INDEXING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = mBackwards ? -kPassiveIndexingVelocity : kPassiveIndexingVelocity;
            break;
        case ZOOMING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = mBackwards ? -kZoomingVelocity : kZoomingVelocity;
            break;
        case SLOW_ZOOMING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = (mBackwards ? -kZoomingVelocity : kZoomingVelocity) * 0.3;
            break;
        case HELLA_ZOOMING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = (mBackwards ? -kZoomingVelocity : kZoomingVelocity) * 1.2;
            break;
        default:
            System.out.println("Fell through on Indexer states!");
        }
    }

    public double getIndexerVelocity() {
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            return mPeriodicIO.indexer_demand;
        } else {
            return 0;
        }
    }

    public void setState(WantedAction wanted_state) {
        final State prev_state = mState;
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case PASSIVE_INDEX:
            mState = State.PASSIVE_INDEXING;
            break;
        case ZOOM:
            mState = State.ZOOMING;
            break;
        case SLOW_ZOOM:
            mState = State.SLOW_ZOOMING;
            break;
        case HELLA_ZOOM:
            mState = State.HELLA_ZOOMING;
            break;
        }

        if (mState != prev_state && mState == State.PASSIVE_INDEXING) {
            mBackwards = !mBackwards;
        }

        if (mState != prev_state && mState == State.ZOOMING) {
            mMaster.configClosedloopRamp(0.2, 0);
        } else if (mState != prev_state) {
            mMaster.configClosedloopRamp(0.0, 0);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.indexer_velocity = mMaster.getSelectedSensorVelocity(0) * 600. / 2048. / kGearRatio;
        mPeriodicIO.indexer_current = mMaster.getStatorCurrent();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            if (mPeriodicIO.indexer_current > kJamCurrent && Timer.getFPGATimestamp() - mLastCurrentSpikeTime > kCurrentIgnoreTime) {
                mBackwards = !mBackwards;
                mLastCurrentSpikeTime = Timer.getFPGATimestamp();
            }
            mMaster.selectProfileSlot(1, 0);
            mMaster.set(mPeriodicIO.indexer_control_mode, (mPeriodicIO.indexer_demand / 600.0) * kGearRatio * 2048.0);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
