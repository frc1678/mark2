package com.team1678.frc2021.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team1678.frc2021.Constants;
import com.team254.lib.drivers.BaseTalonChecker;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends ServoMotorSubsystem {
    private static Hood mInstance;
    private boolean mHoming = true;

    private final double kCalibratingVoltage = -2;

    public synchronized static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood(Constants.HoodConstants.kHoodServoConstants);
        }

        return mInstance;
    }

    private Hood(final ServoMotorSubsystemConstants constants) {
        super(constants);
        mMaster.setSelectedSensorPosition((int) unitsToTicks(17.66));
    }

    @Override
    public synchronized boolean atHomingLocation() {
        return Canifier.getInstance().getHoodLimit();
    }

    public synchronized boolean isHoming() {
        return mHoming;
    }
    
    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean getAtGoal() {
        return Util.epsilonEquals(getAngle(), getSetpoint(), 5.0);
    }

    public synchronized boolean getTucked() {
        return Util.epsilonEquals(getAngle(), this.mConstants.kMinUnitsLimit, 5.0); 
    }

    public synchronized void calibrateHood() {
        mControlState = ControlState.MOTION_MAGIC;
        mMaster.setSelectedSensorPosition((int) unitsToTicks(17.66));
        mMaster.overrideSoftLimitsEnable(true);
        mHoming = false;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mHoming) {
            if (mControlState == ControlState.OPEN_LOOP) {
                mMaster.set(ControlMode.PercentOutput, kCalibratingVoltage);
            } else {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
            }
        } else {
            super.writePeriodicOutputs();
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();

        if (mHoming) {
            mControlState = ControlState.OPEN_LOOP;
            
            if (mPeriodicIO.master_stator_current > 15 /*|| atHomingLocation()*/) {
                calibrateHood();
            }

        }
    }

    @Override
    public boolean checkSystem() {
        return BaseTalonChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<BaseTalon>>() {
            private static final long serialVersionUID = -716113039054569446L;

            {
                add(new MotorChecker.MotorConfig<>("master", mMaster));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mRunOutputPercentage = 0.5;
                mRunTimeSec = 1.0;
                mCurrentFloor = 0.1;
                mRPMFloor = 90;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 200;
                mRPMSupplier = () -> mMaster.getSelectedSensorVelocity();
            }
        });
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        SmartDashboard.putBoolean(mConstants.kName + " Calibrated", !mHoming);
        SmartDashboard.putString("Hood Control State", mControlState.toString());
        SmartDashboard.putBoolean("Hood at Homing Location", atHomingLocation());
        SmartDashboard.putNumber("Hood Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Hood Current", mPeriodicIO.master_stator_current);
    }

    public void setCoastMode() {
        mMaster.setNeutralMode(NeutralMode.Coast);
    }
}