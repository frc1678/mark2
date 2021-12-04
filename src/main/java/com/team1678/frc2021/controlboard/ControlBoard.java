package com.team1678.frc2021.controlboard;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.controlboard.CustomXboxController.Axis;
import com.team1678.frc2021.controlboard.CustomXboxController.Button;
import com.team1678.frc2021.controlboard.CustomXboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Deadband;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
    private final double kSwerveDeadband = Constants.stickDeadband;
    private final double kOperatorDeadband = 0.15;

    private int mDPadUp = -1;
    private int mDPadDown = -1;
    private int mDPadRight = -1;
    private int mDPadLeft = -1;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;
    private TurretCardinal mLastCardinal;

    private static ControlBoard mInstance = null;

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

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final CustomXboxController driver;
    private final CustomXboxController operator;

    private ControlBoard() {
        driver = new CustomXboxController(0);
        operator = new CustomXboxController(Constants.kButtonGamepadPort);
        reset();
    }

    
    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getController().getRawAxis(1);
        double strafeAxis = driver.getController().getRawAxis(0);

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);


        forwardAxis = Constants.Swerve.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.Swerve.invertXAxis ? strafeAxis :-strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

            double scaled_x = tAxes.x() - (deadband_vector.x()) / (1 - deadband_vector.x());
            double scaled_y = tAxes.y() - (deadband_vector.y()) / (1 - deadband_vector.y());
            return new Translation2d(scaled_x, scaled_y).scale(Constants.Swerve.maxSpeed);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X);
        rotAxis = Constants.Swerve.invertRAxis ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return Constants.Swerve.maxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return driver.getButton(Button.START) && driver.getButton(Button.BACK);
    }

    public Rotation2d getJogTurret() {
        double jogX = -operator.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.X);
        double jogY = -operator.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        
        Translation2d mag = new Translation2d(jogX, jogY);
        Rotation2d turret = mag.direction();

        if (Deadband.inDeadband(mag.norm(), 0.5)) {
            return null;
        }
        return turret;
    }

    public double getJogHood() {
        double jog = operator.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kOperatorDeadband)) {
            return 0.0;
        }
        return (jog - kOperatorDeadband * Math.signum(jog));
    }

    public void setRumble(boolean on) { //TODO: all 5 power cells indexed
        operator.setRumble(on);
    }

    public boolean getSpinUp() {
        return operator.getController().getAButtonPressed();
    }

    public boolean getSpinDown() {
        return operator.getController().getBButtonPressed();
    }

    public boolean getTuck() {
        return operator.getButton(Button.X);
    }

    public boolean getFendorShot() {
        return operator.getController().getStickButtonReleased(Hand.kRight);
    }

    public boolean getUntuck() {
        return operator.getButton(Button.START);
    }

    public boolean getTurretReset() {
        return operator.getController().getBackButtonReleased();
    }

    public boolean getTestSpit() {
        return false;
        // return operator.getController().getStickButtonReleased(Hand.kRight);
    }

    public boolean getRevolve() {
        return operator.getButton(CustomXboxController.Button.X);
    }

    public boolean getShoot() {
        return operator.getController().getYButtonPressed();
    }

    public boolean getPreShot() {
        return operator.getController().getBButtonReleased();
    }
    
    public boolean getIntake() {
        return operator.getTrigger(CustomXboxController.Side.RIGHT);
    } 
    
    public boolean getOuttake(){
        return operator.getTrigger(CustomXboxController.Side.LEFT);
    }

    public boolean getShotUp() {
        int pov = operator.getDPad();

        if (pov != mDPadUp) {
            mDPadUp = pov;
            return pov == 0;
        }
        return false;
    }

    public boolean getShotDown() {
        int pov = operator.getDPad();

        if (pov != mDPadDown) {
            mDPadUp = pov;
            return pov == 180;
        }
        return false;
    }

    public boolean getControlPanelRotation() {
        int pov = operator.getDPad();

        if (pov != mDPadRight) {
            mDPadRight = pov;
            return pov == 90;
        }
        return false;
    }

    public boolean getWantUnjam() {
        return operator.getController().getBumper(Hand.kLeft);
    }

    public boolean getManualZoom() {
        return operator.getController().getBumper(Hand.kRight);
    }

    public boolean getControlPanelPosition() {
        int pov = operator.getDPad();

        if (pov != mDPadLeft) {
            mDPadLeft = pov;
            return pov == 270;
        }
        return false;
    }

    public boolean climbMode() {
        return operator.getButton(CustomXboxController.Button.LB) && operator.getButton(CustomXboxController.Button.RB)  && 
        operator.getTrigger(CustomXboxController.Side.LEFT) &&  operator.getTrigger(CustomXboxController.Side.RIGHT);
    }

    public boolean getArmExtend() {
        return operator.getController().getAButtonPressed();
    }

    public boolean getStopClimb() {
        return operator.getController().getStickButtonReleased(Hand.kRight); 
    }

    public boolean getStopExtend() {
        System.out.println(operator.getController().getStickButton(Hand.kLeft));
        return operator.getController().getStickButtonReleased(Hand.kLeft);
    }

    public boolean getBuddyDeploy() {
        return operator.getController().getBackButtonReleased();
    }

    public int getClimberJog(){
        int povread = operator.getController().getPOV();
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
        int povread = operator.getController().getPOV();
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
        return //operator.getController().getStickButton(Hand.kLeft);
        operator.getButton(CustomXboxController.Button.L_JOYSTICK);
    }

    public boolean getWantHoodScan() {
        return operator.getButton(CustomXboxController.Button.L_JOYSTICK);
    }

    public boolean getManualArmRetract() {
        return //operator.getController().getStickButton(Hand.kRight);
        operator.getButton(CustomXboxController.Button.R_JOYSTICK);
    }

    public boolean getClimb() {
        return operator.getController().getYButtonReleased();
    }

    public boolean getBrake() {
        return false; // operator.getController().getYButtonReleased();
    }

    public boolean getWrangle() {
        return operator.getButton(CustomXboxController.Button.X);
    }

    public boolean getLeaveClimbMode() {
        return operator.getButton(Button.BACK) && operator.getButton(Button.START);
    }

    public void reset() {
        mLastCardinal = TurretCardinal.NONE;
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    public TurretCardinal getTurretCardinal() {
        int dPad = operator.getDPad();
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
        return operator.getTrigger(CustomXboxController.Side.LEFT);
    }

    // Intake
    public boolean getRunIntake() {
        return operator.getTrigger(Side.RIGHT);
    }

    public boolean getRetractIntake() {
        return operator.getTrigger(Side.LEFT);
    }
    

}