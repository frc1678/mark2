package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Indexer;
import com.team1678.frc2021.subsystems.Superstructure;


/**
 * Waits until the shooter and feeder are up to speed, used befoer shooting
 * 
 * @see Action
 */
public class WaitForSpinupAction implements Action {
    private Superstructure mSuperstrucure = Superstructure.getInstance();
    

    public WaitForSpinupAction() {
    }

    @Override
    public boolean isFinished() {
        return Indexer.getInstance().getState() == Indexer.State.ZOOMING;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
    }
}