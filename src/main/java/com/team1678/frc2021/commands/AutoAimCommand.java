package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAimCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final double mTurretAngle;
    private final double mStartDelay;
    private double startTime;
    private boolean isFinished = false;

    public AutoAimCommand(Superstructure superstructure, double turretAngle, double startDelay) {
        mSuperstructure = superstructure;
        mTurretAngle = turretAngle;
        mStartDelay = startDelay;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        mSuperstructure.setWantTuck(false);
        mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(mTurretAngle));;
    }

    @Override
    public void execute(){
        //if(Timer.getFPGATimestamp() - startTime > mStartDelay){
            isFinished = true;
        //}
    }

    @Override 
    public boolean isFinished(){
        return mSuperstructure.isAimed() || isFinished;
    }
}