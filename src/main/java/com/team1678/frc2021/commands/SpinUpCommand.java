package com.team1678.frc2021.commands;

import java.util.concurrent.DelayQueue;

import com.team1678.frc2021.subsystems.Shooter;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinUpCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final Shooter shooter;
    private final double startDelay;
    private double startTime;
    private boolean isFinished = false;

    public SpinUpCommand(Superstructure superstructure, double delay) {
        mSuperstructure = superstructure;
        shooter = Shooter.getInstance();
        startDelay = delay;
    }

    @Override 
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        //if(Timer.getFPGATimestamp() - startTime > startDelay){
            mSuperstructure.setWantSpinUp(true);
            isFinished = true;
       // }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}
