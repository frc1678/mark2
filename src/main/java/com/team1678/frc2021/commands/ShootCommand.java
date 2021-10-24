package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Trigger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private double startTime = Double.POSITIVE_INFINITY;
    private boolean isFinished = false;

    public ShootCommand(Superstructure superstructure) {
        mSuperstructure = superstructure;
    }

    @Override
    public void initialize() {
        mSuperstructure.setWantShoot(true);
    }

    @Override
    public void execute(){

        if(Trigger.getInstance().getPopoutSolenoid() && startTime == Double.POSITIVE_INFINITY){
            startTime = Timer.getFPGATimestamp();
        }

        if(Timer.getFPGATimestamp() - startTime > 1.5){
            mSuperstructure.setWantShoot(false);
            mSuperstructure.setWantSpinUp(true);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

}
