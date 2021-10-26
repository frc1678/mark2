package com.team1678.frc2021.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.team1678.frc2021.subsystems.Swerve;

public class AutonomousSelector {

    private static SendableChooser<Rotation2d> orientationChooser;
    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");
        orientationChooser = new SendableChooser<>();
        orientationChooser.setDefaultOption("Forward", Rotation2d.fromDegrees(0.0));
        orientationChooser.addOption("Backwards", Rotation2d.fromDegrees(180.0));
        orientationChooser.addOption("Left", Rotation2d.fromDegrees(90.0));
        orientationChooser.addOption("Right", Rotation2d.fromDegrees(270.0));
        autoTab.add("Starting Orientation", orientationChooser);

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.addOption("Test Straight", AutonomousMode.TEST_STRAIGHT_PATH);
        autonomousModeChooser.addOption("Right Eight Near Ball", AutonomousMode.RIGHT_EIGHT_NEAR_BALL);
        autonomousModeChooser.addOption("Right Eight Far Ball", AutonomousMode.RIGHT_EIGHT_FAR_BALL);
        autonomousModeChooser.addOption("Left Eight Near Ball", AutonomousMode.LEFT_EIGHT_NEAR_BALL);
        autonomousModeChooser.addOption("Left Eight Far Ball", AutonomousMode.LEFT_EIGHT_FAR_BALL);
        autonomousModeChooser.addOption("Aiming Test", AutonomousMode.AIM_TEST_AUTO);

        autoTab.add("Mode", autonomousModeChooser);

        
    }

    public Command getCommand(Swerve s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case TEST_STRAIGHT_PATH:
                return new TestStraightPath(s_Swerve);
            case RIGHT_EIGHT_NEAR_BALL:
                return new RightEightNearMode(s_Swerve);
            case RIGHT_EIGHT_FAR_BALL:
                return new RightEightFarMode(s_Swerve);
            case LEFT_EIGHT_NEAR_BALL:
                return new LeftEightNearMode(s_Swerve);
            case LEFT_EIGHT_FAR_BALL:
                return new LeftEightFarMode(s_Swerve);
            case AIM_TEST_AUTO:
                return new AimTestAuto(s_Swerve);
            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }

        return null;
    }

    public AutonomousSelector() {
    }

    private enum AutonomousMode {
        TEST_STRAIGHT_PATH,
        RIGHT_EIGHT_NEAR_BALL,
        RIGHT_EIGHT_FAR_BALL,
        LEFT_EIGHT_NEAR_BALL,
        LEFT_EIGHT_FAR_BALL,
        AIM_TEST_AUTO
    }

}
