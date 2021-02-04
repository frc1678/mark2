/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package com.team1678.frc2021.auto.actions;

import com.team1678.frc2021.subsystems.Swerve;
import com.team254.lib.geometry.Translation2d;

/**
 * 
 */
public class WaitForDistanceAction implements Action{
    Swerve mSwerve;
    Translation2d pointOfInterest;
    double distance;

    public WaitForDistanceAction(Translation2d target, double distance){
        mSwerve = Swerve.getInstance();
        pointOfInterest = target;
        this.distance = distance;
    }

    @Override
    public boolean isFinished() {
        return mSwerve.getPose().getTranslation().distance(pointOfInterest.unwrap()) <= distance;
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
