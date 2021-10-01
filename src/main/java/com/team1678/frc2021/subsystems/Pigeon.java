package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {
	private static Pigeon instance = null;

	public static Pigeon getInstance() {
		if (instance == null) {
			instance = new Pigeon();
		}
		return instance;
	}

	public Pigeon(int canId) {
		pigeon = new PigeonIMU(canId);
	}

	private PigeonIMU pigeon;

	private Pigeon() {
		try {
			pigeon = new PigeonIMU(16);
		} catch (Exception e) {
			System.out.println(e);
		}
	}

	public boolean isGood() {
		return (pigeon.getState() == PigeonState.Ready) ? true : false;
	}

	public Rotation2d getYaw() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		SmartDashboard.putNumber("Pigeon Heading", pigeon.getFusedHeading(fusionStatus));
		return Rotation2d.fromDegrees(pigeon.getFusedHeading(fusionStatus)/*-ypr[0]*/);
	}

	public double getPitch() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[1];
	}

	public double getRoll() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[2];
	}

	public double[] getYPR() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}

	public void setAngle(double angle) {
		pigeon.setFusedHeading(angle * 64.0, 10);
		pigeon.setYaw(angle, 10);
		System.out.println("Pigeon angle set to: " + angle);
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putString("Pigeon Good", pigeon.getState().toString());
		getYaw();
	}

	public void calibrate() {
		pigeon.setFusedHeading(0);

	}

	public Rotation2d getUnadjustedAngle() {
		return Rotation2d.fromRadians(getAxis(Axis.YAW));
	}

	public double getUnadjustedRate() {
		// TODO Auto-generated method stub
		return 0;
	}

	public double getAxis(Axis axis) {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        switch (axis) {
            case PITCH:
                return Math.toRadians(ypr[1]);
            case ROLL:
                return Math.toRadians(ypr[2]);
            case YAW:
                return Math.toRadians(ypr[0]);
            default:
                return 0.0;
        }
    }

	public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}
