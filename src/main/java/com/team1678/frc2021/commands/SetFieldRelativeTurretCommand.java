package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetFieldRelativeTurretCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final double mTurretAngle;

    public SetFieldRelativeTurretCommand(Superstructure superstructure, double turretAngle) {
        mSuperstructure = superstructure;
        mTurretAngle = turretAngle;
    }

    @Override
    public void initialize() {
        mSuperstructure.setWantTuck(false);
        mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(mTurretAngle));;
    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}