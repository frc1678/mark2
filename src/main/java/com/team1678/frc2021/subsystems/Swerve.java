package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.Ports;
import com.team1678.frc2021.SwerveModule;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Swerve extends Subsystem {

    private static Swerve mInstance;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;

    public boolean isSnapping;
    public ProfiledPIDController snapPidController;


    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    public Swerve() {
        gyro = new PigeonIMU(Ports.PIGEON);
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw());
        snapPidController = new ProfiledPIDController(Constants.SnapConstants.snapKP,
                                              Constants.SnapConstants.snapKI, 
                                              Constants.SnapConstants.snapKD,
                                              Constants.SnapConstants.kThetaControllerConstraints);
        snapPidController.enableContinuousInput(-Math.PI, Math.PI);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Odometry Pose X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Pose Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Pose Rot", swerveOdometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putBoolean("Is Snapping", isSnapping);
        SmartDashboard.putNumber("Pigeon Heading", getYaw().getDegrees());
        SmartDashboard.putNumber("Snap Target", Math.toDegrees(snapPidController.getGoal().position));
        for(SwerveModule mod : mSwerveMods){
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", MathUtil.inputModulus(mod.getCanCoder().getDegrees() - mod.angleOffset, 0, 360));
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    public void teleopDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if(isSnapping) {
            if(Math.abs(rotation) == 0.0) {
                maybeStopSnap(false);
                rotation = calculateSnapValue();
            } else {
                maybeStopSnap(true);
            }
        } 
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
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
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber(mod.moduleNumber + " rot", swerveModuleStates[mod.moduleNumber].angle.getDegrees());
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public double calculateSnapValue() {
        return snapPidController.calculate(getYaw().getRadians());
    }

    public void startSnap(double snapAngle) {
        snapPidController.reset(getYaw().getRadians());
        snapPidController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
        isSnapping = true;
    }
    
    TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    private boolean snapComplete() {
        double error = snapPidController.getGoal().position - getYaw().getRadians();
        return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.snapEpsilon), Constants.SnapConstants.snapTimeout);
        //return Math.abs(error) < Math.toRadians(Constants.SnapConstants.snapEpsilon);
    }

    public void maybeStopSnap(boolean force){
        if(!isSnapping){
            return;
        } 
        if(force || snapComplete()) {
            isSnapping = false;
            snapPidController.reset(getYaw().getRadians());
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
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

    public void resetAnglesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
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


    @Override
    public void zeroSensors(){
        zeroGyro();
    }
    
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void zeroGyro(double reset){
        gyro.setYaw(reset);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public void updateSwerveOdometry(){
        swerveOdometry.update(getYaw(), getStates());  
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
