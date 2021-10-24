package com.team1678.frc2021.commands;

import com.team1678.frc2021.subsystems.Hood;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TuckCommand extends CommandBase{

    private final Superstructure mSuperstructure;
    private final boolean mTuck;

    public TuckCommand(Superstructure superstructure, boolean tuck) {
        mSuperstructure = superstructure;
        mTuck = tuck;
    }
    
    @Override
    public void initialize() {
        mSuperstructure.setWantTuck(mTuck);
    }

    @Override
    public boolean isFinished(){
        return Hood.getInstance().getTucked();
    }
}