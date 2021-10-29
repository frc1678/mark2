package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Skywalker extends Subsystem {

    private static Skywalker mInstance;

    public synchronized static Skywalker getInstance() {
        if (mInstance == null) {
            mInstance = new Skywalker();
        }
        return mInstance;
    }
    
    public enum WantedAction {
        NONE, SHIFT_RIGHT, SHIFT_LEFT,
    }
    
    public enum State {
        IDLE, SHIFTING_RIGHT, SHIFTING_LEFT,
    }

    private State mState = State.IDLE;

    private final TalonSRX mMaster;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double current;

        // OUTPUTS
        public double demand;
    }

    private Skywalker() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kSkywalkerMasterId);
        mMaster.changeMotionControlFramePeriod(100);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 100);
        mMaster.setNeutralMode(NeutralMode.Brake);
    }

    public synchronized State getState() {
        return mState;
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = Constants.kIdleVoltage;
                break;
            case SHIFTING_RIGHT:
                mPeriodicIO.demand = Constants.kShiftingRightVoltage;
                break;
            case SHIFTING_LEFT:
                mPeriodicIO.demand = Constants.kShiftingLeftVoltage;
                break;
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case SHIFT_RIGHT:
                mState = State.SHIFTING_RIGHT;
                break;
            case SHIFT_LEFT:
                mState = State.SHIFTING_LEFT;
                break;
        }
    }    

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // startLogging();
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Skywalker.this) {
                    runStateMachine();

                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                mMaster.neutralOutput();
                stop();
                stopLogging();
            }
        });
    }

    public synchronized void startLogging() {
        
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    @Override
    public void zeroSensors() {}

    @Override
    public void stop() {
        mState = State.IDLE;
        mPeriodicIO.demand = 0.0;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void writeToLog() {}

    @Override
    public void outputTelemetry(){
        SmartDashboard.putNumber("SKywalker Current", mPeriodicIO.current);
        SmartDashboard.putString("Skywalker State", mState.toString());
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }   
    }
}