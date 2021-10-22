package com.team1678.frc2021.commands;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.RobotContainer;
import com.team1678.frc2021.Constants.SnapConstants;
import com.team1678.frc2021.subsystems.Swerve;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CardinalSnapCommand extends CommandBase {
    
    private Swerve s_Swerve;
    private double targetAngle;
    private boolean fieldRelative;
    private boolean openLoop;

    private ProfiledPIDController controller;
    private TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    public CardinalSnapCommand(Swerve s_Swerve, double targetDeg, boolean fieldRelative, boolean openLoop){

        this.setName(targetDeg + "_SNAP");

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.targetAngle = Math.toRadians(targetDeg);
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        controller = new ProfiledPIDController(
            Constants.SnapConstants.snapKP,
            Constants.SnapConstants.snapKI,
            Constants.SnapConstants.snapKD,
            Constants.SnapConstants.kThetaControllerConstraints
        );

        controller.enableContinuousInput(0.0, 2 * Math.PI);
        controller.reset(s_Swerve.getPose().getRotation().getRadians());
        controller.setGoal(new TrapezoidProfile.State(Math.toRadians(targetDeg), 0.0));
    }

    @Override
    public void execute(){
        double timestamp = Timer.getFPGATimestamp();

        double[] axes = RobotContainer.getInstance().getTeleopSwerve().getAxes();;

        double yAxis = -axes[0];
        double xAxis = -axes[1];

        Translation2d translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);


        double currentAngle = s_Swerve.getPose().getRotation().getRadians();
        double rotation = controller.calculate(currentAngle);
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }

    
    @Override
    public boolean isFinished(){
        double error = targetAngle - s_Swerve.getPose().getRotation().getRadians();
        return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.snapEpsilon), Constants.SnapConstants.snapTimeout);
    }
}
