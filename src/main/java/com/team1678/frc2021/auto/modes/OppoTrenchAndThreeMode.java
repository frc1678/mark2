package com.team1678.frc2021.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team1323.lib.trajectory.Trajectory;
import com.team1323.lib.trajectory.timing.TimedState;
import com.team1678.frc2021.auto.AutoModeEndedException;

public class OppoTrenchAndThreeMode extends AutoModeBase {

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.startToOppoTrench, trajectories.oppoTrenchToStart);
    }

    @Override
    protected void routine() throws AutoModeEndedException {

    }
    
}