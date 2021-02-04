package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Indexer;

/**
 * Waits until indexer rotates given theta before continuing action
 * 
 * @see Action
 */
public class WaitForIndexerSpinAction implements Action {

    private Indexer mIndexer = Indexer.getInstance();

    private double mTheta; // how many degrees on the indexer
    private double mStartTheta;

    public WaitForIndexerSpinAction(double theta) {
        mTheta = theta;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mIndexer.getIndexerTheta() - mStartTheta) >= mTheta;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        mStartTheta = mIndexer.getIndexerTheta();
    }
}
