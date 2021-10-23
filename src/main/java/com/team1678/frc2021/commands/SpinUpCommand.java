package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Shooter;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinUpCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final Shooter shooter;

    public SpinUpCommand(Superstructure superstructure) {
        mSuperstructure = superstructure;
        shooter = Shooter.getInstance();
    }

    @Override
    public void execute() {;
        mSuperstructure.setWantSpinUp(true);
        if(shooter.spunUp()){
            end(false);
        }

    }

    @Override
    public boolean isFinished() {
        return shooter.spunUp();
    }

}
