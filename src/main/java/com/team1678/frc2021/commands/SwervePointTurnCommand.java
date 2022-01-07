package com.team1678.frc2021.commands;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwervePointTurnCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;
  private final Supplier<Rotation2d> m_desiredRotation;


  public SwervePointTurnCommand(Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
      PIDController xController, PIDController yController, ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation, Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements){ 

    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");

    m_controller = new HolonomicDriveController(
        requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
        requireNonNullParam(yController, "xController", "SwerveControllerCommand"),
        requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));

    m_outputModuleStates = requireNonNullParam(outputModuleStates, "frontLeftOutput", "SwerveControllerCommand");

    m_desiredRotation = requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    var targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), new Pose2d(m_pose.get().getX(), m_pose.get().getY(), m_desiredRotation.get()), 0.0, m_desiredRotation.get());
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_pose.get().getRotation().minus(m_desiredRotation.get()).getDegrees()) < 10.0;
  }
}
