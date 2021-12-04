package com.team1678.frc2021.controlboard;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.CustomXboxController.Button;
import com.team1678.frc2021.controlboard.CustomXboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Deadband;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class GamepadButtonControlBoard {
    private final double kDeadband = 0.15;

    private int mDPadUp = -1;
    private int mDPadDown = -1;
    private int mDPadRight = -1;
    private int mDPadLeft = -1;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;
    private TurretCardinal mLastCardinal;

    private static GamepadButtonControlBoard mInstance = null;

    // Turret
    public enum TurretCardinal {
        BACK(180),
        FRONT(0),
        LEFT(90),
        RIGHT(-90),
        NONE(0),
        FRONT_LEFT(30, 45),
        FRONT_RIGHT(-30, -45),
        BACK_LEFT(150, 135),
        BACK_RIGHT(210, 235);

        public final Rotation2d rotation;
        private final Rotation2d inputDirection;

        TurretCardinal(double degrees) {
            this(degrees, degrees);
        }

        TurretCardinal(double degrees, double inputDirectionDegrees) {
            rotation = Rotation2d.fromDegrees(degrees);
            inputDirection = Rotation2d.fromDegrees(inputDirectionDegrees);
        }

        public static TurretCardinal findClosest(double xAxis, double yAxis) {
            return findClosest(new Rotation2d(yAxis, -xAxis, true));
        }

        public static TurretCardinal findClosest(Rotation2d stickDirection) {
            var values = TurretCardinal.values();

            TurretCardinal closest = null;
            double closestDistance = Double.MAX_VALUE;
            for (int i = 0; i < values.length; i++) {
                var checkDirection = values[i];
                var distance = Math.abs(stickDirection.distance(checkDirection.inputDirection));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closest = checkDirection;
                }
            }
            return closest;
        }

        public static boolean isDiagonal(TurretCardinal cardinal) {
            return cardinal == FRONT_LEFT || cardinal == FRONT_RIGHT || cardinal == BACK_LEFT || cardinal == BACK_RIGHT;
        }
    }

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final CustomXboxController mController;

    private GamepadButtonControlBoard() {
        mController = new CustomXboxController(Constants.kButtonGamepadPort);
        reset();
    }

    public Rotation2d getJogTurret() {
        double jogX = -mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.X);
        double jogY = -mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        
        Translation2d mag = new Translation2d(jogX, jogY);
        Rotation2d turret = mag.direction();

        if (Deadband.inDeadband(mag.norm(), 0.5)) {
            return null;
        }
        return turret;
    }

    public double getJogHood() {
        double jog = mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    public boolean getSpinUp() {
        return mController.getController().getAButtonPressed();
    }

    public boolean getSpinDown() {
        return mController.getController().getBButtonPressed();
    }

    public boolean getTuck() {
        return mController.getButton(Button.X);
    }

    public boolean getFendorShot() {
        return mController.getController().getStickButtonReleased(Hand.kRight);
    }

    public boolean getUntuck() {
        return mController.getButton(Button.START);
    }

    public boolean getTurretReset() {
        return mController.getController().getBackButtonReleased();
    }

    public boolean getTestSpit() {
        return false;
        // return mController.getController().getStickButtonReleased(Hand.kRight);
    }

    public boolean getRevolve() {
        return mController.getButton(CustomXboxController.Button.X);
    }

    public boolean getShoot() {
        return mController.getController().getYButtonPressed();
    }

    public boolean getPreShot() {
        return mController.getController().getBButtonReleased();
    }
    
    public boolean getIntake() {
        return mController.getTrigger(CustomXboxController.Side.RIGHT);
    } 
    
    public boolean getOuttake(){
        return mController.getTrigger(CustomXboxController.Side.LEFT);
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
            mDPadUp = pov;
            return pov == 180;
        }
        return false;
    }

    public boolean getControlPanelRotation() {
        int pov = mController.getDPad();

        if (pov != mDPadRight) {
            mDPadRight = pov;
            return pov == 90;
        }
        return false;
    }

    public boolean getWantUnjam() {
        return mController.getController().getBumper(Hand.kLeft);
    }

    public boolean getManualZoom() {
        return mController.getController().getBumper(Hand.kRight);
    }

    public boolean getControlPanelPosition() {
        int pov = mController.getDPad();

        if (pov != mDPadLeft) {
            mDPadLeft = pov;
            return pov == 270;
        }
        return false;
    }

    public boolean climbMode() {
        return mController.getButton(CustomXboxController.Button.LB) && mController.getButton(CustomXboxController.Button.RB)  && 
        mController.getTrigger(CustomXboxController.Side.LEFT) &&  mController.getTrigger(CustomXboxController.Side.RIGHT);
    }

    public boolean getArmExtend() {
        return mController.getController().getAButtonPressed();
    }

    public boolean getStopClimb() {
        return mController.getController().getStickButtonReleased(Hand.kRight); 
    }

    public boolean getStopExtend() {
        System.out.println(mController.getController().getStickButton(Hand.kLeft));
        return mController.getController().getStickButtonReleased(Hand.kLeft);
    }

    public boolean getBuddyDeploy() {
        return mController.getController().getBackButtonReleased();
    }

    public int getClimberJog(){
        int povread = mController.getController().getPOV();
        switch(povread){
            case 0:
                return 1;
            case 180:
                return -1;
            default:
                return 0;
        }
    }
    /**
     * The skywalker input
     *
     * Uses a getPOV() call to read d-pad, and returns -1 for left, 0 for neutral, and +1 for right.
     * 
     * @return the inputted skywalker direction, -1 for left, 0 for neutral, +1 for right
     */
    public int getSkywalker(){
        int povread = mController.getController().getPOV();
        switch(povread){
            case 90:
                return 1;
            case 270:
                return -1;
            default:
                return 0;
        }
    }

    public boolean getManualArmExtend() {
        return //mController.getController().getStickButton(Hand.kLeft);
        mController.getButton(CustomXboxController.Button.L_JOYSTICK);
    }

    public boolean getWantHoodScan() {
        return mController.getButton(CustomXboxController.Button.L_JOYSTICK);
    }

    public boolean getManualArmRetract() {
        return //mController.getController().getStickButton(Hand.kRight);
        mController.getButton(CustomXboxController.Button.R_JOYSTICK);
    }

    public boolean getClimb() {
        return mController.getController().getYButtonReleased();
    }

    public boolean getBrake() {
        return false; // mController.getController().getYButtonReleased();
    }

    public boolean getWrangle() {
        return mController.getButton(CustomXboxController.Button.X);
    }

    public boolean getLeaveClimbMode() {
        return mController.getButton(Button.BACK) && mController.getButton(Button.START);
    }

    public void reset() {
        mLastCardinal = TurretCardinal.NONE;
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    public TurretCardinal getTurretCardinal() {
        int dPad = mController.getDPad();
        TurretCardinal newCardinal = dPad == -1 ? TurretCardinal.NONE
                : TurretCardinal.findClosest(Rotation2d.fromDegrees(-dPad - 180.0));
        if (newCardinal != TurretCardinal.NONE && TurretCardinal.isDiagonal(newCardinal)) {
            // Latch previous direction on diagonal presses, because the D-pad sucks at
            // diagonals.
            newCardinal = mLastCardinal;
        }
        boolean valid = mDPadValid.update(Timer.getFPGATimestamp(), newCardinal != TurretCardinal.NONE
                && (mLastCardinal == TurretCardinal.NONE || newCardinal == mLastCardinal));
        if (valid) {
            if (mLastCardinal == TurretCardinal.NONE) {
                mLastCardinal = newCardinal;
            }
            return mLastCardinal;
        } else {
            mLastCardinal = newCardinal;
        }
        return TurretCardinal.NONE;
    }

    public boolean getAutoAim() {
        return mController.getTrigger(CustomXboxController.Side.LEFT);
    }

    // Not used
    // public double getJoggingX() {
    //     double jog = mController.getJoystick(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.X);
    //     if (Deadband.inDeadband(jog, kDeadband)) {
    //         return 0.0;
    //     }
    //     return (jog - kDeadband * Math.signum(jog));
    // }

    // public double getJoggingZ() {
    //     double jog = mController.getJoystick(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.Y);
    //     if (Deadband.inDeadband(jog, kDeadband)) {
    //         return 0.0;
    //     }
    //     return (jog - kDeadband * Math.signum(jog));
    // }

    // Intake
    public boolean getRunIntake() {
        return mController.getTrigger(Side.RIGHT);
    }

    public boolean getRetractIntake() {
        return mController.getTrigger(Side.LEFT);
    }
    

}