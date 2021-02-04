package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.RobotState;

public class WaitUntilSeesTargetAction implements Action {
    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getAimingParameters(false, -1, Constants.kMaxGoalTrackAge).isPresent();
    }

    @Override
    public void done() {}
}