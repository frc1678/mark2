package com.team1678.frc2021.commands;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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

        this.setName("TELEOP_SWERVE");

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    public double[] getAxes() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        double[] axes = {yAxis, xAxis, rAxis};

        return axes;
    }

    @Override
    public void execute() {
        double[] axes = getAxes();

        double yAxis = -axes[0];
        double xAxis = -axes[1];
        double rAxis = -axes[2];

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
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
