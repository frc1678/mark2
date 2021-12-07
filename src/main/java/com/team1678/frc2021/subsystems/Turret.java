package com.team1678.frc2021.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team1678.frc2021.Constants;
import com.team1678.lib.util.HallCalibration;
import com.team254.lib.drivers.BaseTalonChecker;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.LatchedBoolean;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends ServoMotorSubsystem {
    private static Turret mInstance;
    private LatchedBoolean mJustReset = new LatchedBoolean();
    private boolean mHoming = true;
    public static final boolean kUseManualHomingRoutine = false;
    private HallCalibration calibration = new HallCalibration(0);
    private double mOffset = 0;
    private DigitalInput mLimitSwitch = new DigitalInput(1);

    private static final SupplyCurrentLimitConfiguration CURR_LIM = new SupplyCurrentLimitConfiguration(true, 40, 60, 0.01);

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret(Constants.TurretConstants.kTurretServoConstants);
        }
        return mInstance;
    }

    private Turret(final ServoMotorSubsystemConstants constants) {
        super(constants);

        mMaster.setSelectedSensorPosition(0);   
        mMaster.configSupplyCurrentLimit(CURR_LIM);
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean atHomingLocation() {
        final double enc = mMaster.getSelectedSensorPosition(0);
        calibration.update(enc, !mLimitSwitch.get());
        if (calibration.isCalibrated()) {
            mOffset = enc + calibration.getOffset();
            return true;
        }
        return false;
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (mJustReset.update(reset) && kUseManualHomingRoutine) {
            System.out.println("Turret going into home mode!");
            mHoming = true;
            mMaster.overrideSoftLimitsEnable(false);
        }
    }

    public synchronized boolean safeToIntake() {
        Rotation2d angle = Rotation2d.fromDegrees(getAngle());
        if (angle.getDegrees() < 55 && angle.getDegrees() > -55) {
            return false;
        }
        return true;
    }

    public synchronized boolean isHoming() {
        return mHoming;
    }

    public synchronized void cancelHoming() {
        if (mHoming) {
            mMaster.setSelectedSensorPosition(0);
        }
        mHoming = false;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mHoming) {
            if (mControlState == ControlState.OPEN_LOOP) {
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, 0.0);
            } else {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
            }
        } else {
            super.writePeriodicOutputs();
        }
        // mMaster.set(ControlMode.PercentOutput, 0.3);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        if (mHoming) {
            if (atHomingLocation() && !mHasBeenZeroed) {
                mMaster.setSelectedSensorPosition((int) Math.floor(mOffset));
                mMaster.overrideSoftLimitsEnable(true);
                System.out.println("Homed!!!");
                mHoming = false;
                mHasBeenZeroed = true;
            }
        }
    }

    @Override
    public boolean checkSystem() {
        return BaseTalonChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<BaseTalon>>() {
            private static final long serialVersionUID = 1636612675181038895L;

            {
                add(new MotorChecker.MotorConfig<>("master", mMaster));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mRunOutputPercentage = 0.1;
                mRunTimeSec = 1.0;
                mCurrentFloor = 0.1;
                mRPMFloor = 90;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 200;
                mRPMSupplier = mMaster::getSelectedSensorVelocity;
            }
        });
    }

    public boolean getHoming() {
        return mHoming;
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        SmartDashboard.putBoolean(mConstants.kName + " Calibrated", !mHoming);
    }

    public void setCoastMode() {
        mMaster.setNeutralMode(NeutralMode.Coast);
    }

    
}