package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReadyGyro extends CommandBase{

    private final  Swerve mSwerve;

    public ReadyGyro(Swerve swerve) {
        mSwerve = swerve;
    }

    @Override
    public void initialize() {
        mSwerve.zeroGyro(mSwerve.getYaw().getDegrees());
    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}