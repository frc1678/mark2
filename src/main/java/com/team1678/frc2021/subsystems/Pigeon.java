package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.team1678.frc2021.Constants;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {
	private static Pigeon instance = null;
	public static Pigeon getInstance() {
		if(instance == null){
			instance = new Pigeon();
		}
		return instance;
	}
	
	private PigeonIMU pigeon;
    
	private Pigeon() {
		try{
			pigeon = new PigeonIMU(Constants.kPigeonIMUId);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	
	public boolean isGood() {
		return (pigeon.getState() == PigeonState.Ready) ? true : false;
	}
	
	public Rotation2d getYaw() {
		if(RobotBase.isReal()){
			PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
			return Rotation2d.fromDegrees(-pigeon.getFusedHeading(fusionStatus));
		}
		return new Rotation2d();
	}

	public double getPitch() {
		double [] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[1];
	}

	public double getRoll() {
		double [] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[2];
	}

	public double[] getYPR() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}
	
	public void setAngle(double angle){
		pigeon.setFusedHeading(-angle * 64.0, 10);
		pigeon.setYaw(-angle, 10);
		System.out.println("Pigeon angle set to: " + angle);
	}
	
	public void outputToSmartDashboard() {
		SmartDashboard.putString("Pigeon Good", pigeon.getState().toString());
	}
}
