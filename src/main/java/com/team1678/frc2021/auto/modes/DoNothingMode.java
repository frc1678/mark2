package com.team1678.frc2021.auto.modes;

import com.team1678.frc2021.auto.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("doing nothing");
    }
}
