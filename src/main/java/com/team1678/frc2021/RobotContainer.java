// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2021;

import com.team1678.frc2021.auto.exampleAuto;
import com.team1678.frc2021.commands.CardinalSnapCommand;
import com.team1678.frc2021.commands.TeleopSwerve;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Superstructure mSuperstructure = Superstructure.getInstance();

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Button zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final Button yButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final Button bButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final Button aButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private final Button xButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final Button rightBumper = new JoystickButton(driver, XboxController.Button.kBumperRight.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  private static RobotContainer instance;
  public static RobotContainer getInstance(){
    if(instance == null){
      instance = new RobotContainer();
    }
    return instance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  public TeleopSwerve getTeleopSwerve() {
    boolean fieldRelative = true;
    boolean openLoop = true;

    return new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    rightBumper.whenPressed(new InstantCommand(() -> mSuperstructure.setWantTuck(true)));
    
    // Snap Commands
    yButton.whenPressed(new CardinalSnapCommand(s_Swerve, 0, true, true));
    bButton.whenPressed(new CardinalSnapCommand(s_Swerve, 90, true, true));
    aButton.whenPressed(new CardinalSnapCommand(s_Swerve, 180, true, true));
    xButton.whenPressed(new CardinalSnapCommand(s_Swerve, 270, true, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}