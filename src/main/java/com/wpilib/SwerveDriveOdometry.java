/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.wpilib;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.team1323.lib.geometry.UnwrappablePose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * swerve drive encoders and swerve azimuth encoders.
 *
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
public class SwerveDriveOdometry {
  private final SwerveDriveKinematics m_kinematics;
  private UnwrappablePose2d m_poseMeters;
  private double m_prevTimeSeconds = -1;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;

  /**
   * Constructs a SwerveDriveOdometry object.
   *
   * @param kinematics  The swerve drive kinematics for your drivetrain.
   * @param gyroAngle   The angle reported by the gyroscope.
   * @param initialPose The starting position of the robot on the field.
   */
  public SwerveDriveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
      UnwrappablePose2d initialPose) {
    m_kinematics = kinematics;
    m_poseMeters = initialPose;
    m_gyroOffset = m_poseMeters.wrap().getRotation().rotateBy(gyroAngle.unwrap().toNegative().wrap());
    m_previousAngle = initialPose.getRotation().wrap();
    HAL.report(tResourceType.kResourceType_Odometry, tInstances.kOdometry_SwerveDrive);
  }

  /**
   * Constructs a SwerveDriveOdometry object with the default pose at the origin.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   * @param gyroAngle  The angle reported by the gyroscope.
   */
  public SwerveDriveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle) {
    this(kinematics, gyroAngle, new UnwrappablePose2d());
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>
   * The gyroscope angle does not need to be reset here on the user's robot code.
   * The library automatically takes care of offsetting the gyro angle.
   *
   * @param pose      The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  public void resetPosition(UnwrappablePose2d pose, Rotation2d gyroAngle) {
    m_poseMeters = pose;
    m_previousAngle = pose.getRotation().wrap();
    m_gyroOffset = m_poseMeters.wrap().getRotation().rotateBy(gyroAngle.unwrap().toNegative().wrap());
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public UnwrappablePose2d getPoseMeters() {
    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method takes in the current time as a
   * parameter to calculate period (difference between two timestamps). The period
   * is used to calculate the change in distance from a velocity. This also takes
   * in an angle parameter which is used instead of the angular rate that is
   * calculated from forward kinematics.
   *
   * @param currentTimeSeconds The current time in seconds.
   * @param gyroAngle          The angle reported by the gyroscope.
   * @param moduleStates       The current state of all swerve modules. Please
   *                           provide the states in the same order in which you
   *                           instantiated your SwerveDriveKinematics.
   * @return The new pose of the robot.
   */
  public UnwrappablePose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle,
      SwerveModuleState... moduleStates) {
    double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
    m_prevTimeSeconds = currentTimeSeconds;

    var angle = gyroAngle.rotateBy(m_gyroOffset);

    var chassisState = m_kinematics.toChassisSpeeds(moduleStates);
    var newPose = UnwrappablePose2d
        .exp(new Twist2d(chassisState.vxMetersPerSecond * period, chassisState.vyMetersPerSecond * period,
            // angle.minus(m_previousAngle).getRadians()));
            angle.rotateBy(m_previousAngle.unwrap().toNegative().wrap()).getRadians()).wrap());

    m_previousAngle = angle;
    m_poseMeters = new UnwrappablePose2d(newPose.getTranslation(), angle.unwrap());

    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method automatically calculates the
   * current time to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity. This also
   * takes in an angle parameter which is used instead of the angular rate that is
   * calculated from forward kinematics.
   *
   * @param gyroAngle    The angle reported by the gyroscope.
   * @param moduleStates The current state of all swerve modules. Please provide
   *                     the states in the same order in which you instantiated
   *                     your SwerveDriveKinematics.
   * @return The new pose of the robot.
   */
  public UnwrappablePose2d update(Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
    return updateWithTime(Timer.getFPGATimestamp(), gyroAngle, moduleStates);
  }
}
