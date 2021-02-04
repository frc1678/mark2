package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.wpilibj.Timer;

public class WaitToFinishPathAction implements Action{
	Swerve mSwerve;
	double timeout;
	double startTime;
	
	public WaitToFinishPathAction(){
		mSwerve = Swerve.getInstance();
		timeout = 15.0;
	}
	
	public WaitToFinishPathAction(double timeout){
		mSwerve = Swerve.getInstance();
		this.timeout = timeout;
	}
	
	@Override
	public boolean isFinished(){
		return mSwerve.hasFinishedPath() || ((Timer.getFPGATimestamp() - startTime) > timeout);
	}
	
	@Override
	public void start(){
		startTime = Timer.getFPGATimestamp();
	}
	
	@Override
	public void update(){
	}
	
	@Override
	public void done(){
	}
}
