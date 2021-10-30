package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;

public class Trigger extends Subsystem {
    private static Trigger mInstance;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonFX mTrigger;
    private final Solenoid mPopoutSolenoid;

    private final Indexer mIndexer = Indexer.getInstance();

    private boolean mCurrentLimitTriggered = false;
    private double mCurrentLimitTimer = 0.0;

    private boolean mRunningManual = false;

    private static double kTriggerVelocityConversion = 600.0 / 2048.0;
    private static double kUnjamTime = 0.5;
    private static double kJamCurrent = 150.0;
    private static double kUnjamSpeed = -1000.0;
    
    private static double kTriggerTolerance = 200.0;
    private static final StatorCurrentLimitConfiguration CURR_LIM = new StatorCurrentLimitConfiguration(true, 40, 40, 0.3);

    private Trigger() {
        mTrigger = TalonFXFactory.createDefaultTalon(Constants.kTriggerWheelID);
        mTrigger.changeMotionControlFramePeriod(255);
        mTrigger.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mTrigger.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

        mTrigger.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mTrigger.set(ControlMode.PercentOutput, 0);
        mTrigger.setInverted(false); //TODO: check value
        mTrigger.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mTrigger.enableVoltageCompensation(true);
        mTrigger.configStatorCurrentLimit(CURR_LIM);

        mTrigger.config_kP(0, Constants.kTriggerP, Constants.kLongCANTimeoutMs);
        mTrigger.config_kI(0, Constants.kTriggerI, Constants.kLongCANTimeoutMs);
        mTrigger.config_kD(0, Constants.kTriggerD, Constants.kLongCANTimeoutMs);
        mTrigger.config_kF(0, Constants.kTriggerF, Constants.kLongCANTimeoutMs);

        mPopoutSolenoid = Constants.makeSolenoidForId(Constants.kTriggerPopoutSolenoidID);
    }

    public synchronized static Trigger mInstance() {
        if (mInstance == null) {
            mInstance = new Trigger();
        }
        return mInstance;
    }

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/TRIGGER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // startLogging();
            }
            @Override
            public void onLoop(double timestamp) {
            }
            @Override
            public void onStop(double timestamp) {
                stopLogging();
            }
        });
    }

    public synchronized void setOpenLoop(double trigger) {
        mPeriodicIO.trigger_demand = trigger;
        mRunningManual = true;
    }

    
    public synchronized boolean getPopoutSolenoid() {
        return mPeriodicIO.popout_solenoid;
    }

    public synchronized double getTriggerRPM() {
        return mTrigger.getSelectedSensorVelocity();
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.trigger_demand > 0) {
            return Util.epsilonEquals(mPeriodicIO.trigger_demand, mPeriodicIO.trigger_velocity, kTriggerTolerance);
            // return mPeriodicIO.trigger_demand <= -550.0;
        }
        return false;
    }

    boolean spunUp = spunUp();

    public synchronized void setPopoutSolenoid(boolean popout) {
        mPeriodicIO.popout_solenoid = popout;
    }

    public synchronized void setVelocity(double setpoint) {
        mPeriodicIO.trigger_demand = setpoint;
        mRunningManual = false;
    }

    public synchronized void currentSafety() {
        final double now = Timer.getFPGATimestamp();

        if (mPeriodicIO.trigger_current > kJamCurrent && !mCurrentLimitTriggered) {
            mCurrentLimitTriggered = true;
            mCurrentLimitTimer = now;
        } else if (mCurrentLimitTriggered && now - mCurrentLimitTimer < kUnjamTime) {
            mPeriodicIO.trigger_demand = kUnjamSpeed;
            mPeriodicIO.popout_solenoid = true;
        } else if (now - mCurrentLimitTimer > kUnjamTime){
            mCurrentLimitTriggered = false;
        }
    }

    public synchronized boolean getJammed() {
        return mCurrentLimitTriggered;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.trigger_velocity = mTrigger.getSelectedSensorVelocity() * kTriggerVelocityConversion;
        mPeriodicIO.trigger_voltage = mTrigger.getMotorOutputVoltage();
        mPeriodicIO.trigger_current = mTrigger.getStatorCurrent();
        mPeriodicIO.trigger_temperature = mTrigger.getTemperature();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (!mRunningManual) {
            currentSafety();
            mTrigger.set(ControlMode.Velocity, mPeriodicIO.trigger_demand / kTriggerVelocityConversion);
            // mPopoutSolenoid.set(true);
            mPopoutSolenoid.set(mPeriodicIO.popout_solenoid);
        } else {
            mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.trigger_demand);
            // mPopoutSolenoid.set(true);
            mPopoutSolenoid.set(mPeriodicIO.popout_solenoid);
        }
    }
    
    @Override
    public synchronized boolean checkSystem() {
        return true;
    }

    public static Trigger getInstance() {
        if (mInstance == null) {
            mInstance = new Trigger();
        }
        return mInstance;
    }

    public static class PeriodicIO {
        // INPUTS
        public double trigger_velocity;
        public double trigger_current;
        public double trigger_voltage;
        public double trigger_temperature;
        
        //OUTPUTS
        public double trigger_demand;
        public boolean popout_solenoid;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Trigger Velocity", mPeriodicIO.trigger_velocity);
        SmartDashboard.putNumber("Trigger Current", mPeriodicIO.trigger_current);
        SmartDashboard.putNumber("Trigger Goal", mPeriodicIO.trigger_demand);
        SmartDashboard.putNumber("Trigger Temperature", mPeriodicIO.trigger_temperature);
        SmartDashboard.putBoolean("Trigger Spun Up: ", spunUp());

        SmartDashboard.putBoolean("Popout Solenoid", mPeriodicIO.popout_solenoid);
        SmartDashboard.putBoolean("jam", mCurrentLimitTriggered);
        
        if (mCSVWriter != null) {
            mCSVWriter.write();
            System.out.println("LOGGING trigger");
        }
    }
}