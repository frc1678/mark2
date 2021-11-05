package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReadyGyro extends CommandBase{

    private final Swerve mSwerve;
    private double adjustedGyroRead;

    public ReadyGyro(Swerve swerve) {
        mSwerve = swerve;
    }

    @Override
    public void initialize() {
        adjustedGyroRead = mSwerve.getYaw().getDegrees() - 180;
    }

    @Override 
    public void execute(){
        mSwerve.zeroGyro(adjustedGyroRead);
    }

}