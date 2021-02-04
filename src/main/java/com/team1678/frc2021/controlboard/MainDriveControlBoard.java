package com.team1678.frc2021.controlboard;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.CustomXboxController.Button;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class MainDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }

        return mInstance;
    }

    private final CustomXboxController mController;

    private int mDPadUp = -1;
    private int mDPadDown = -1;

    private MainDriveControlBoard() {
        mController = new CustomXboxController(Constants.kDriverJoystickPort);
    }

    public double getSwerveYInput() {
        return mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.X);
    }

    public double getSwerveXInput() {
        return -mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
    }

    public double getSwerveRotation() {
        return mController.getJoystick(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.X);
    }

    public boolean getSnapNorth() {
        return mController.getController().getYButtonPressed();
    }

    public boolean getSnapEast() {
        return mController.getController().getBButtonPressed();
    }

    public boolean getSnapSouth() {
        return mController.getController().getAButtonPressed();
    }

    public boolean getSnapWest() {
        return mController.getController().getXButtonPressed();
    }

    public boolean getResetGyro() {
        return mController.getButton(Button.BACK);
    }

    public boolean getLowPowerDrive() {
        return false; // TODO: create a held down in custom xbox controller
    }

    public boolean getTuck() {
        return mController.getTrigger(CustomXboxController.Side.RIGHT) || mController.getTrigger(CustomXboxController.Side.LEFT);
    }

    public boolean getManualFastRoller() {
        return mController.getController().getBumper(Hand.kRight);
    }

    public boolean getManualSlowRoller() {
        return mController.getController().getBumper(Hand.kLeft);
    }

    public boolean getStopManualRoller() {
        return mController.getController().getBumperReleased(Hand.kLeft) || mController.getController().getBumperReleased(Hand.kRight);
    }

    public boolean getShotUp() {
        int pov = mController.getDPad();

        if (pov != mDPadUp) {
            mDPadUp = pov;
            return pov == 0;
        }
        return false;
    }
    
    public boolean getShotDown() {
        int pov = mController.getDPad();

        if (pov != mDPadDown) {
            mDPadDown = pov;
            return pov == 180;
        }
        return false;
    }
}