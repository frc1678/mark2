package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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

public class Climber extends Subsystem  {
    private static Climber mInstance = null;

    private static final double kIdleVoltage = 0.0;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private TimeDelayedBoolean mShiftSolenoidTimer = new TimeDelayedBoolean();

    public enum WantedAction {
        NONE, JOG_UP, JOG_DOWN,
    }

    private static WantedAction mWantedAction = WantedAction.NONE;

    public enum State {
        IDLE, JOGGING_UP, JOGGING_DOWN,
    }

    private State mState = State.IDLE;

    private final TalonFX mMaster;
    private final Solenoid mShiftSolenoid;

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 80, 80, 1.0);
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;


    private Climber() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIntakeRollerId);
        mMaster.changeMotionControlFramePeriod(60);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 60, 100);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(true);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mMaster.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(30000, Constants.kLongCANTimeoutMs);
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
        SmartDashboard.putNumber("ClimbOutputVoltage", mMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("Climber Voltage", mPeriodicIO.voltage);
        SmartDashboard.putNumber("ClimberPosition", mPeriodicIO.position);
        SmartDashboard.putNumber("ClimberVelocity", mPeriodicIO.velocity);
        SmartDashboard.putNumber("Climber Current", mPeriodicIO.current);
        SmartDashboard.putBoolean("Shifter Goal", mPeriodicIO.shift_solenoid);
        SmartDashboard.putBoolean("Shifter Actual", mPeriodicIO.shift_out);
        SmartDashboard.putString("Climber Wanted Action", mWantedAction.name());
        SmartDashboard.putNumber("Climber Goal", mPeriodicIO.demand);

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

    public void setShift(boolean shift) {
        mPeriodicIO.shift_solenoid = shift;
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                break;
            case JOGGING_UP:
                mPeriodicIO.shift_solenoid = true;
                mPeriodicIO.demand = 12.0;
                mPeriodicIO.brake_solenoid = false;
                break;
            case JOGGING_DOWN:
                mPeriodicIO.shift_solenoid = true;
                mPeriodicIO.demand = -12.0;
                mPeriodicIO.brake_solenoid = false;
                break;
            default:
                System.out.println("Fell through on Climber states!");
        }
    }

    public void setState(WantedAction wanted_state) {
        mWantedAction = wanted_state;
        switch (wanted_state) {
            case NONE:
                  mState = State.IDLE;
                break;
            case JOG_UP:
                mState = State.JOGGING_UP;
                break;
            case JOG_DOWN:
                mState = State.JOGGING_DOWN;
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
        mPeriodicIO.current = mMaster.getStatorCurrent();
        mPeriodicIO.voltage = mMaster.getMotorOutputVoltage();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mState == State.JOGGING_DOWN || mState == State.JOGGING_UP) {
            mShiftSolenoid.set(mPeriodicIO.shift_solenoid);
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand > 12.0 ? 12.0/12.0 : mPeriodicIO.demand/12.0);
        } else {
            mShiftSolenoid.set(mPeriodicIO.shift_solenoid);
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
        public boolean brake_solenoid;
    }
}