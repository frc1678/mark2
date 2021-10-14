package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class Climber extends Subsystem  {
    private static Climber mInstance = null;

    private static final double kIdleVoltage = 0.0;
    private static final double kExtendVoltage = 2.0;
    private static final double kClimbVoltage = -4.0;
    private static final double kBrakeVelocity = 500.0;
    private double mInitialTime;

    private static final int kExtendDelta = (2) - (174100);
    private static final int kHugDelta = (157000 - (-27600));
    private static final int kClimbDelta = 10000;

    private static boolean wasShifted;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private TimeDelayedBoolean mShiftSolenoidTimer = new TimeDelayedBoolean();

    public enum WantedAction {
        NONE, EXTEND, HUG, CLIMB, BRAKE, STOP, JOG_UP, JOG_DOWN,
    }

    private static WantedAction mWantedAction = WantedAction.NONE;

    public enum State {
        IDLE, EXTENDING, HUGGING, CLIMBING, BRAKING, JOGGING_UP, JOGGING_DOWN,
    }

    private State mState = State.IDLE;

    private final TalonFX mMaster;
    private final Solenoid mShiftSolenoid;
    
    private double mHoldingPos = 0.0;
    private double mZeroPos;
    private boolean mExtended = false;
    private TimeDelayedBoolean brake_activation = new TimeDelayedBoolean();

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 40, 40, .2);
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;


    private Climber() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIntakeRollerId);
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(true);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mMaster.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(0, 0.5);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 0);
        mMaster.config_kF(0, 0.05);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);

        mMaster.setNeutralMode(NeutralMode.Coast);

        mMaster.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);

        mShiftSolenoid = Constants.makeSolenoidForId(Constants.kShiftSolenoidId);
    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public synchronized void setBrakeMode(boolean brake) {
        mMaster.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("ClimberState", mState.name());
        SmartDashboard.putNumber("ClimbVoltage", mPeriodicIO.demand);
        SmartDashboard.putNumber("ClimberPosition", mPeriodicIO.position);
        SmartDashboard.putNumber("ClimberVelocity", mPeriodicIO.velocity);
        SmartDashboard.putBoolean("Shifter Goal", mPeriodicIO.shift_solenoid);
        SmartDashboard.putBoolean("Shifter Actual", mPeriodicIO.shift_out);
        SmartDashboard.putString("Climber Wanted Action", mWantedAction.name());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
                startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Climber.this) {
                    runStateMachine();

                    if (mPeriodicIO.shift_out) {
                        wasShifted = true;
                    }

                    if (wasShifted && !mPeriodicIO.shift_out) {    
                        mPeriodicIO.shift_solenoid = true;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stopLogging();
            }
        });
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/CLIMBER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public synchronized State getState() {
        return mState;
    }

    public void setBrake(boolean brake) {
        mPeriodicIO.brake_solenoid = brake;
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public void setZeroPosition() {
        mZeroPos = mPeriodicIO.position;
    }

    public void setShift(boolean shift) {
        mPeriodicIO.shift_solenoid = shift;
    }

    public void runStateMachine() {
        final double now = Timer.getFPGATimestamp();
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                mPeriodicIO.arm_solenoid = true;
                break;
            case EXTENDING:
                mPeriodicIO.demand = mZeroPos + kExtendDelta;
                if ((mPeriodicIO.position - mZeroPos) > kExtendDelta - 2000) {
                    mExtended = true;
                }
                mPeriodicIO.arm_solenoid = true;
                mPeriodicIO.brake_solenoid = false;
                break;
            case JOGGING_UP:
               // if (mExtended) {
                    mPeriodicIO.demand = mPeriodicIO.position + 20000;
                //}
                mPeriodicIO.arm_solenoid = true;
                mPeriodicIO.brake_solenoid = false;
                break;
            case JOGGING_DOWN:
                //if (mExtended) {
                    mPeriodicIO.demand = mPeriodicIO.position - 20000;
               // }
                mPeriodicIO.arm_solenoid = true;
                mPeriodicIO.brake_solenoid = false;
                break;
            case CLIMBING:
                mPeriodicIO.demand = mZeroPos + kClimbDelta;
                mPeriodicIO.arm_solenoid = true;
                mPeriodicIO.brake_solenoid = false;

                if ((Math.abs(mPeriodicIO.position - (mZeroPos + kClimbDelta)) < 5000 && Math.abs(mPeriodicIO.velocity) < kBrakeVelocity)) {
                    mHoldingPos = mPeriodicIO.position;
                    mState = State.BRAKING;
                }
                mPeriodicIO.brake_solenoid = false;
                break;
            case BRAKING:
                mPeriodicIO.demand = mHoldingPos;
                mPeriodicIO.arm_solenoid = true;
                if (!mPeriodicIO.brake_solenoid) {
                    if (mPeriodicIO.velocity < kBrakeVelocity) {
                        mPeriodicIO.brake_solenoid = true;
                    }
                }
                break;
            default:
                System.out.println("Fell through on Climber states!");
        }
    }

    public void setState(WantedAction wanted_state) {
        if (wanted_state == WantedAction.BRAKE && mState != State.BRAKING) {
            mHoldingPos = mPeriodicIO.position;
        }

        mWantedAction = wanted_state;

        switch (wanted_state) {
            case NONE:
                  mState = State.IDLE;
                break;
            case EXTEND:
                mState = State.EXTENDING;
                break;
            case HUG:
                mState = State.HUGGING;
                break;
            case JOG_UP:
                mState = State.JOGGING_UP;
                break;
            case JOG_DOWN:
                mState = State.JOGGING_DOWN;
                break;
            case CLIMB:
                mState = State.CLIMBING;
                break;
            case BRAKE:
                mState = State.BRAKING;
                break;
            case STOP:
                mState = State.IDLE;
                break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.shift_out = mShiftSolenoidTimer.update(mShiftSolenoid.get(), 0.2);
        mPeriodicIO.position = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.velocity = mMaster.getSelectedSensorVelocity(0);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
        mPeriodicIO.current = mMaster.getSupplyCurrent();
        mPeriodicIO.voltage = mMaster.getMotorOutputVoltage();
        //LogSend();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mShiftSolenoid.set(mPeriodicIO.shift_solenoid);
        if (mState == State.BRAKING || mState == State.EXTENDING || mState == State.HUGGING || mState == State.CLIMBING || mState == State.JOGGING_DOWN || mState == State.JOGGING_UP) {
            mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand);
        } else {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public double position;
        public double velocity;
        public double current;
        public double voltage;
        public boolean shift_out;

        // OUTPUTS
        public double demand;
        public boolean shift_solenoid;
        public boolean arm_solenoid;
        public boolean brake_solenoid;
    }
}