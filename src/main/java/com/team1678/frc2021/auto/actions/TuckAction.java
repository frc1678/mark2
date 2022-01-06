package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Hood;
import com.team1678.frc2021.subsystems.Superstructure;

public class TuckAction implements Action {

    private final Superstructure mSuperstructure;
    private final boolean mTuck;

    public TuckAction(Superstructure superstructure, boolean tuck) {
        mSuperstructure = superstructure;
        mTuck = tuck;
    }

    @Override
    public void start() {
        mSuperstructure.setWantTuck(mTuck);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Hood.getInstance().getTucked();
    }

    @Override
    public void done() {}
}
