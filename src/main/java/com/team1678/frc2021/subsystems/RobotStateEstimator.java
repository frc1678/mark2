package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.RobotState;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

public class RobotStateEstimator extends Subsystem {
	RobotState mRobotState = RobotState.getInstance();
	Swerve mSwerve;
	Turret turret;

    private static RobotStateEstimator instance = null;
	public static RobotStateEstimator getInstance() {
		if(instance == null)
			instance = new RobotStateEstimator();
		return instance;
	}
	
	RobotStateEstimator() {
	}

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
		mSwerve = Swerve.getInstance();
		turret = Turret.getInstance();
        }

        @Override
        public synchronized void onLoop(double timestamp) {
			mRobotState.addFieldToVehicleObservation(timestamp, mSwerve.getPose().wrap());
        }

        @Override
        public void onStop(double timestamp) {
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {

    }
}