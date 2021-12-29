package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Superstructure;

public class SpinUpAction implements Action {

    private final Superstructure mSuperstructure;

    public SpinUpAction(Superstructure superstructure) {
        mSuperstructure = superstructure;
    }

    @Override
    public void start() {
        mSuperstructure.setWantSpinUp(true);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}
