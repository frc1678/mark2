package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.logger.LogStorage;
import com.team1678.frc2021.logger.LoggingSystem;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Wrangler extends Subsystem {
    public static double kWrangleVoltage = -10.;
    public static double kHoldingVoltage = 0.;
    
    private static Wrangler mInstance;
    private PeriodicOutputs mPeriodicOutputs = new PeriodicOutputs();
    private final TalonFX mMaster;
    private final Solenoid mDeployer;

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 30, 30, .2);
    private boolean mBuddyClimb = false;

    public enum WantedAction {
        NONE, DEPLOY, WRANGLE, RETRACT,
    }

    public enum State {
        IDLE, DEPLOYING, WRANGLING, RETRACTING,
    }

    private State mState = State.IDLE;

    public synchronized static Wrangler getInstance() {
        if (mInstance == null) {
            mInstance = new Wrangler();
        }
        return mInstance;
    }

    private Wrangler() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kWranglerId);
        mDeployer = Constants.makeSolenoidForId(Constants.kWranglerSolenoidId);

        mMaster.set(ControlMode.PercentOutput, 0.);
        mMaster.setInverted(false);
        mMaster.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.configOpenloopRamp(.5);
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("WranglerState", mState.name());
        SmartDashboard.putNumber("WranglerMotorSetpoint", mPeriodicOutputs.demand);
        SmartDashboard.putBoolean("WranglerOut", getWranglerOut());
    }

    @Override
    public void stop() {
        mPeriodicOutputs.demand = kHoldingVoltage;
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
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Wrangler.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
            }
        });
    }

    public synchronized boolean getWranglerOut() {
        return mPeriodicOutputs.deployer_solenoid;
    }

    public void runStateMachine() {
        switch (mState) {
        case IDLE:
            mPeriodicOutputs.demand = kHoldingVoltage;
            break;
        case DEPLOYING:
            mPeriodicOutputs.demand = kHoldingVoltage;
            mPeriodicOutputs.deployer_solenoid = true;
            break;
        case WRANGLING:
            mPeriodicOutputs.demand = kWrangleVoltage;
            mPeriodicOutputs.deployer_solenoid = true;
            break;
        case RETRACTING:
            mPeriodicOutputs.demand = kHoldingVoltage;
            mPeriodicOutputs.deployer_solenoid = false;
            break;
        default:
            System.out.println("Fell through on Wrangler states!");
        }
    }

    public void forceRetract() {
        mPeriodicOutputs.deployer_solenoid = false;
    }

    public double getVoltage() {
        return mPeriodicOutputs.demand;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case DEPLOY:
            mState = State.DEPLOYING;
            mBuddyClimb = true;
            break;
        case WRANGLE:
            mState = State.WRANGLING;
            break;
        case RETRACT:
            mState = State.RETRACTING;
            break;
        default:
            System.out.println("No Wrangler state!");
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicOutputs.demand / 12.0);
        mDeployer.set(mPeriodicOutputs.deployer_solenoid);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public static class PeriodicOutputs {
        // OUTPUTS
        public double demand;
        public boolean deployer_solenoid;
    }
}
