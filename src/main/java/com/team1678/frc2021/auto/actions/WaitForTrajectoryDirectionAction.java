/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;
import com.team254.lib.geometry.Rotation2d;

/**
 * 
 */
public class WaitForTrajectoryDirectionAction implements Action{
    Rotation2d lowerBound, upperBound;
    Swerve mSwerve;
    public WaitForTrajectoryDirectionAction(Rotation2d lower, Rotation2d upper){
        lowerBound = lower;
        upperBound = upper;
        mSwerve = Swerve.getInstance();
    }

    @Override
    public boolean isFinished() {
        double direction = mSwerve.getLastTrajectoryVector().direction().getDegrees();
        return lowerBound.getDegrees() <= direction && direction <= upperBound.getDegrees();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}
