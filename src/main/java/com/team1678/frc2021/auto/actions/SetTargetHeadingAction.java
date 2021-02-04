package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;

public class SetTargetHeadingAction extends RunOnceAction{
	double targetHeading;
	Swerve mSwerve;
	
	public SetTargetHeadingAction(double targetHeading){
		this.targetHeading = targetHeading;
		mSwerve = Swerve.getInstance();
	}
	
	@Override
	public void runOnce() {
		mSwerve.setAbsolutePathHeading(targetHeading);
	}

}
