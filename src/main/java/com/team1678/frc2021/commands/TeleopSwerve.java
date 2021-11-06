package com.team1678.frc2021.commands;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    private double applyRotationalDeadband(double input){
        double deadband = Constants.stickDeadband;
        if (Math.abs(input) < deadband) {
            return 0.0;
        } else {
            return (input - (Math.signum(input) * deadband)) / (1 - deadband);
        }
    }

    private Translation2d applyTranslationalDeadband(Translation2d input) {
        double deadband = Constants.stickDeadband;
        if (Math.abs(input.getNorm()) < deadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(input.getX(), input.getY());
            Translation2d deadband_vector = new Translation2d(deadband, deadband_direction);

            double scaled_x = input.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double scaled_y = input.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());
            return new Translation2d(scaled_x, scaled_y);
        }

    }

    public double[] getAxes() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);

        Translation2d tAxes;
        
        /* Deadbands */
        tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
        rAxis = applyRotationalDeadband(rAxis);

        double[] axes = {tAxes.getX(), tAxes.getY(), rAxis};

        return axes;
    }

    @Override
    public void execute() {

        double yAxis;
        double xAxis;
        double rAxis;

        Translation2d tAxes; // translational axis

        /* Inversions */
        yAxis = Constants.Swerve.invertYAxis ? controller.getRawAxis(translationAxis) : -controller.getRawAxis(translationAxis);
        xAxis = Constants.Swerve.invertXAxis ? controller.getRawAxis(strafeAxis) : -controller.getRawAxis(strafeAxis);
        rAxis = Constants.Swerve.invertRAxis ? controller.getRawAxis(rotationAxis) : -controller.getRawAxis(rotationAxis);

        /* Deadbands */
        tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
        rAxis = applyRotationalDeadband(rAxis);

        translation = new Translation2d(tAxes.getX(), tAxes.getY()).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

        SmartDashboard.putNumber("X Controller Input", translation.getX());
        SmartDashboard.putNumber("Y Controller Input", translation.getY());
        SmartDashboard.putNumber("Rot Controller Input", rotation);

    }

    public Translation2d getChassisTranslation() {
        double[] axes = getAxes();

        double yAxis = axes[0];
        double xAxis = axes[1];

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);

        return translation;
    }

    public double getChassisRotation() {
        double[] axes = getAxes();

        double rAxis = axes[2];
        
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;

        return rotation;
    }

}
