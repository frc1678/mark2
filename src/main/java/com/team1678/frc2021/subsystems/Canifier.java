package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.team1678.frc2021.Constants;

public class Canifier extends Subsystem {
    private static Canifier mInstance;
    private CANifier mCanifier;
    private PeriodicInputs mPeriodicInputs;

    private Canifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 255, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        mPeriodicInputs = new PeriodicInputs();
    }

    public synchronized static Canifier getInstance() {
        if (mInstance == null) {
            mInstance = new Canifier();
        }
        return mInstance;
    }

    public int getWristTicks() {
        return mCanifier.getQuadraturePosition();
    }

    public synchronized boolean getIndexerLimit() {
        return mPeriodicInputs.indexer_limit_;
    }

    public synchronized boolean getTurretLimit() {
        return mPeriodicInputs.turret_limit_;
    }

    public synchronized boolean getHoodLimit() {
        return mPeriodicInputs.hood_limit_;
    }

    public synchronized boolean getFrontProxy() {
        return mPeriodicInputs.front_proxy_;
    }

    public synchronized boolean getRightProxy() {
        return mPeriodicInputs.right_proxy_;
    }

    public synchronized boolean getLeftProxy() {
        return mPeriodicInputs.left_proxy_;
    }

    public synchronized boolean getBackRightProxy() {
        return mPeriodicInputs.back_right_proxy_;
    }

    public synchronized boolean getBackLeftProxy() {
        return mPeriodicInputs.back_left_proxy_;
    }

    public synchronized void resetWristEncoder() {
        mCanifier.setQuadraturePosition(0, 0 );
    }

    public int getDeviceId() {
        return mCanifier.getDeviceID();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        CANifier.PinValues pins = new CANifier.PinValues();
        mCanifier.getGeneralInputs(pins);

        mPeriodicInputs.indexer_limit_ = !pins.SDA;
        mPeriodicInputs.turret_limit_ = !pins.LIMR;
        mPeriodicInputs.hood_limit_ = !pins.SPI_MOSI_PWM1;

        mPeriodicInputs.front_proxy_ = pins.SPI_CLK_PWM0;
        mPeriodicInputs.right_proxy_ = pins.SPI_MOSI_PWM1;
        mPeriodicInputs.left_proxy_ =  pins.SPI_CS_PWM3;
        mPeriodicInputs.back_right_proxy_ = pins.SPI_MISO_PWM2;
        mPeriodicInputs.back_left_proxy_ = pins.QUAD_IDX;
        
    }

    public synchronized CANifier getCanifier() {
        return mCanifier;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {
        mPeriodicInputs = new PeriodicInputs();
    }

    private static class PeriodicInputs {
        public boolean indexer_limit_;
        public boolean turret_limit_;
        public boolean hood_limit_;
        public boolean front_proxy_;
        public boolean right_proxy_;
        public boolean left_proxy_;
        public boolean back_right_proxy_;
        public boolean back_left_proxy_;
    }
}
