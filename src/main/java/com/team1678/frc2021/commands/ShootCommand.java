package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCommand extends CommandBase{

    private final Superstructure mSuperstructure;

    public ShootCommand(Superstructure superstructure) {
        mSuperstructure = superstructure;

    }

    @Override
    public void execute() {
        mSuperstructure.setWantShoot();
    }

}
