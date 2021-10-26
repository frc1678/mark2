package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAimCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final double mTurretAngle;

    public AutoAimCommand(Superstructure superstructure, double turretAngle) {
        mSuperstructure = superstructure;
        mTurretAngle = turretAngle;
    }

    @Override
    public void initialize() {
        mSuperstructure.setWantTuck(false);
        mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(mTurretAngle));;
    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}