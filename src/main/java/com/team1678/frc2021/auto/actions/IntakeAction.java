package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Intake;
import com.team1678.frc2021.subsystems.Superstructure;

public class IntakeAction implements Action {

    private final Intake mIntake;
    private final Superstructure mSuperstructure;

    public IntakeAction(Intake intake, Superstructure superstructure) {
        mIntake = intake;
        mSuperstructure = superstructure;
    }
    
    @Override
    public void start() {}

    @Override
    public void update() {
        mIntake.setState(Intake.WantedAction.INTAKE);
        mSuperstructure.enableIndexer(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}
