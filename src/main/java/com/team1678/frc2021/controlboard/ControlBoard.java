package com.team1678.frc2021.controlboard;

import com.team1678.frc2021.controlboard.GamepadButtonControlBoard.TurretCardinal;
import com.team254.lib.geometry.Rotation2d;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    private CustomXboxController mController;
    
    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final GamepadButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    public void reset() {
    }
    
    public boolean getManualZoom() {
        return mButtonControlBoard.getManualZoom();
    }

    public boolean getWantUnjam() {
        return mButtonControlBoard.getWantUnjam();
    }

    public boolean getShotUp() {
        return mButtonControlBoard.getShotUp();
    }
    
    public boolean getShotDown() {
        return mButtonControlBoard.getShotDown();
    }
 
    public boolean getRunIntake() {
        return mButtonControlBoard.getRunIntake();
    }

    public boolean getRetractIntake() {
        return mButtonControlBoard.getRetractIntake();
    }

    public Rotation2d getJogTurret() {
        return mButtonControlBoard.getJogTurret();
    }

    public double getJogHood() {
        return mButtonControlBoard.getJogHood();
    }

    public boolean getWantHoodScan() {
        return mButtonControlBoard.getWantHoodScan();
    }

    // Intake

    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    public boolean getPreShot() {
        return mButtonControlBoard.getPreShot();
    }

    public boolean getRevolve() {
        return mButtonControlBoard.getRevolve();
    }

    public boolean getSpinUp() {
        return mButtonControlBoard.getSpinUp();
    }

    public boolean getSpinDown() {
        return mButtonControlBoard.getSpinDown();
    }

    public boolean getTuck() {
        return mButtonControlBoard.getTuck() /*|| mDriveControlBoard.getTuck()*/;
    }

    public boolean getUntuck() {
        return mButtonControlBoard.getUntuck();
    }

    public boolean getFendorShot() {
        return mButtonControlBoard.getFendorShot();
    }

    public boolean getTestSpit() {
        return mButtonControlBoard.getTestSpit();
    }

    public boolean getControlPanelRotation() {
        return mButtonControlBoard.getControlPanelRotation();
    }

    public boolean getControlPanelPosition() {
        return mButtonControlBoard.getControlPanelPosition();
    }

    public boolean getTurretReset() {
        return mButtonControlBoard.getTurretReset();
    }

    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    public boolean getArmExtend() {
        return mButtonControlBoard.getArmExtend();
    }

    public boolean getStopExtend() {
        return mButtonControlBoard.getStopExtend();
    }

    public boolean getStopClimb() {
        return mButtonControlBoard.getStopClimb();
    }

    public boolean getBuddyDeploy() {
        return mButtonControlBoard.getBuddyDeploy();
    }

    public int getClimberJog() {
        return mButtonControlBoard.getClimberJog();
    }

    public boolean getManualArmExtend() {
        return mButtonControlBoard.getManualArmExtend();
    }

    public boolean getManualArmRetract() {
        return mButtonControlBoard.getManualArmRetract();
    }

    public boolean getClimb() {
        return mButtonControlBoard.getClimb();
    }

    public boolean getBrake() {
        return mButtonControlBoard.getBrake();
    }

    public boolean getWrangle() {
        return mButtonControlBoard.getWrangle();
    }

    public int getSkywalker(){
        return mButtonControlBoard.getSkywalker();
    }
    
    public boolean climbMode() {
        return mButtonControlBoard.climbMode();
        //return mController.getButton(XboxController.Button.LB) && mController.getButton(XboxController.Button.RB)  && 
        //mController.getTrigger(XboxController.Side.LEFT) &&  mController.getTrigger(XboxController.Side.RIGHT);
    }

    public TurretCardinal getTurretCardinal() {
        return mButtonControlBoard.getTurretCardinal();
    }

    public boolean getLeaveClimbMode() {
        return mButtonControlBoard.getLeaveClimbMode(); 
    }
}
   