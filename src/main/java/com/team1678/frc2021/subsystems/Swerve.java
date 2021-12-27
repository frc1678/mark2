package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import com.team1678.frc2021.SwerveModule;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team1678.frc2021.Constants;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    public boolean isSnapping;
    public ProfiledPIDController snapPidController;
    private double lastSnapInput;
    
    //Instance declaration
	private static Swerve instance = null;
	public static Swerve getInstance() {
		if(instance == null)
			instance = new Swerve();
		return instance;
    }

    public Swerve() {
        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());
        snapPidController = new ProfiledPIDController(Constants.SnapConstants.snapKP,
                                              Constants.SnapConstants.snapKI, 
                                              Constants.SnapConstants.snapKD,
                                              Constants.SnapConstants.kThetaControllerConstraints);
        snapPidController.enableContinuousInput(-Math.PI, Math.PI);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if(isSnapping) {
            if(Math.abs(rotation) == 0.0) {
                maybeStopSnap(false);
                rotation = calculateSnapVectors();
            } else {
                maybeStopSnap(true);
            }
        } 

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public double calculateSnapVectors(){
        return snapPidController.calculate(getYaw().getRadians(), lastSnapInput);
    }

    public void startSnap(double snapAngle){
        lastSnapInput = Math.toRadians(snapAngle);
        snapPidController.reset(getYaw().getRadians());
        //snapPidController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
        isSnapping = true;
    }
    
    TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();
    private boolean snapComplete(){
        double error = lastSnapInput - getYaw().getRadians();
        //return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.snapEpsilon), Constants.SnapConstants.snapTimeout);
        return Math.abs(error) < Math.toRadians(Constants.SnapConstants.snapEpsilon);
    }
    public void maybeStopSnap(boolean force){
        if(!isSnapping){
            return;
        } 
        if(force || snapComplete()){
            isSnapping = false;
            snapPidController.reset(getYaw().getRadians());
        }
    }

    public ChassisSpeeds getChassisVelocity(Translation2d translation, double rotation) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
        );
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void setAnglePIDValues(double kP, double kI, double kD) {
        for (SwerveModule swerveModule : mSwerveMods) {
            swerveModule.updateAnglePID(kP, kI, kD);
        }
    }

    public double[] getAnglePIDValues(int index) {
        return mSwerveMods[index].getAnglePIDValues();
    }

    public void zeroGyro(){
        zeroGyro(0);
    }

    public void zeroGyro(double zeroValue){
        gyro.setYaw(zeroValue);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  

        // for(SwerveModule mod : mSwerveMods){
        //     // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        // }

        // SmartDashboard.putNumber("Pose Translation X", getPose().getX());
        // SmartDashboard.putNumber("Pose Translation Y", getPose().getY());
        // SmartDashboard.putNumber("Pose Rotation", getPose().getRotation().getDegrees());
        // SmartDashboard.putNumber("Gyro Reading", getYaw().getDegrees());
    }
}
