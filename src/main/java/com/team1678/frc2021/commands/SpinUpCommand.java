package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Shooter;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinUpCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final Shooter shooter;
    private boolean isFinished = false;

    public SpinUpCommand(Superstructure superstructure) {
        mSuperstructure = superstructure;
        shooter = Shooter.getInstance();
    }

    @Override 
    public void initialize(){
        mSuperstructure.setWantSpinUp(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
