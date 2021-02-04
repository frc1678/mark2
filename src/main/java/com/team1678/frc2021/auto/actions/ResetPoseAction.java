package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;
import com.team254.lib.geometry.Pose2d;

public class ResetPoseAction extends RunOnceAction{
	private Pose2d newPose;
	boolean leftStartingSide = true;

	Swerve mSwerve;
	
	public ResetPoseAction(Pose2d newPose){
		this.newPose = newPose;
		mSwerve = Swerve.getInstance();
	}

	@Override
	public void runOnce() {
		mSwerve.setStartingPose(newPose.unwrap());
		mSwerve.zeroSensors(newPose.unwrap());
	}

}
