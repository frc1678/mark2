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
        autonomousModeChooser.addOption("Right Ten Far Ball", AutonomousMode.RIGHT_TEN_FAR_BALL);
        autonomousModeChooser.addOption("Left Eight Near Ball", AutonomousMode.LEFT_EIGHT_NEAR_BALL);
        autonomousModeChooser.addOption("Left Eight Far Ball", AutonomousMode.LEFT_EIGHT_FAR_BALL);
        autonomousModeChooser.addOption("Left Six Far Ball", AutonomousMode.LEFT_SIX_FAR_BALL);
        autonomousModeChooser.addOption("Left Six Near Ball", AutonomousMode.LEFT_SIX_NEAR_BALL);
        autonomousModeChooser.addOption("Shot Left Back", AutonomousMode.SHOT_LEFT_BACK);
        autonomousModeChooser.addOption("Shot Left Front", AutonomousMode.SHOT_LEFT_FRONT);
        autonomousModeChooser.addOption("Shot Center Back", AutonomousMode.SHOT_CENTER_BACK);


        autoTab.add("Mode", autonomousModeChooser);

        
    }

    public Command getCommand(Swerve s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case TEST_STRAIGHT_PATH:
                return new TestStraightPath(s_Swerve);
            case RIGHT_TEN_BALL:
                return new RightTenMode(s_Swerve);
            case RIGHT_TEN_NEAR_BALL:
                return new RightTenNearMode(s_Swerve);
            case RIGHT_TEN_FAR_BALL:
                return new RightTenFarMode(s_Swerve);
            case LEFT_EIGHT_NEAR_BALL:
                return new LeftEightNearMode(s_Swerve);
            case LEFT_EIGHT_FAR_BALL:
                return new LeftEightFarMode(s_Swerve);
            case LEFT_SIX_FAR_BALL:
                return new LeftSixFarMode(s_Swerve);
            case LEFT_SIX_NEAR_BALL:
                return new LeftSixNearMode(s_Swerve);
            case SHOT_LEFT_BACK:
                return new ShotLeftBack(s_Swerve);
            case SHOT_LEFT_FRONT:
                return new ShotLeftFront(s_Swerve);
            case SHOT_CENTER_BACK:
                return new ShotCenterBack(s_Swerve);
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
        RIGHT_TEN_BALL,
        RIGHT_TEN_NEAR_BALL,
        RIGHT_TEN_FAR_BALL,
        LEFT_EIGHT_NEAR_BALL,
        LEFT_EIGHT_FAR_BALL,
        LEFT_SIX_FAR_BALL,
        LEFT_SIX_NEAR_BALL,
        SHOT_LEFT_BACK,
        SHOT_LEFT_FRONT,
        SHOT_CENTER_BACK,
        AIM_TEST_AUTO
    }

}
