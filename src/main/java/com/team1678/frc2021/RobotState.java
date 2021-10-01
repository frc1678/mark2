package com.team1678.frc2021;

import com.team1678.frc2021.subsystems.Limelight;
import com.team1678.frc2021.vision.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 100;
    private static final Pose2d kVehicleToTurretFixed = Pose2d.fromTranslation(Constants.kVehicleToTurretTranslation);

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Turret frame: origin is the center of the turret, which is coincident with
     * the origin of the vehicle frame but with potentially different angle.
     *
     * 4. Camera frame: origin is the center of the Limelight imager relative to the
     * turret.
     *
     * 5. Goal frame: origin is the center of the vision target, facing outwards
     * along the normal. Also note that there can be multiple goal frames.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Vehicle-to-turret: Measured by the turret encoder. This is a pure
     * rotation.
     *
     * 3. Turret-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-goal: Measured by the vision system.
     * 
     * 5. Vehicle-to-hood: Measured by bore encoder on hood
     */

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> turret_rotation_;
    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> vehicle_to_hood_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;


    private GoalTracker vision_target_ = new GoalTracker();

    List<com.team254.lib.geometry.Translation2d> mCameraToVisionTargetPoses = new ArrayList<>();

    private RobotState() {
        reset(0.0, new com.team254.lib.geometry.Pose2d(), new Rotation2d(), new Rotation2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle,
            com.team254.lib.geometry.Rotation2d initial_vehicle_to_turret, Rotation2d initial_vehicle_to_hood) {
        reset(start_time, initial_field_to_vehicle);

        turret_rotation_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        turret_rotation_.put(new InterpolatingDouble(start_time), initial_vehicle_to_turret);

        vehicle_to_hood_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        vehicle_to_hood_.put(new InterpolatingDouble(start_time), initial_vehicle_to_hood);
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), new Pose2d(), new Rotation2d(), new Rotation2d());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized double getRobotTheta(double timestamp) {
        return getLatestFieldToVehicle().getValue().getRotation().getDegrees();
    }

    public synchronized Pose2d getVehicleToTurret(double timestamp) {
        return new Pose2d(Constants.kVehicleToTurretTranslation,
                turret_rotation_.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized Rotation2d getVehicleToHood(double timestamp) {
        return vehicle_to_hood_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Translation2d getTurretToLens(double timestamp) {
        final Rotation2d hood_angle = Rotation2d
                .fromDegrees(90 - getVehicleToHood(timestamp).getDegrees() - Constants.kLimelightPitchOffset);
        return new Translation2d(Constants.kHoodToTurret - hood_angle.cos() * Constants.kHoodRadius, 0);
    }

    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Map.Entry<InterpolatingDouble, Rotation2d> getLatestVehicleToTurret() {
        return turret_rotation_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addVehicleToTurretObservation(double timestamp, Rotation2d observation) {
        turret_rotation_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addVehicleToHoodObservation(double timestamp, Rotation2d observation) {
        vehicle_to_hood_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose2d displacement, Twist2d measured_velocity,
            Twist2d predicted_velocity) {
        distance_driven_ += displacement.getTranslation().x();
        addFieldToVehicleObservation(timestamp,
               getLatestFieldToVehicle().getValue().transformBy(displacement));
        vehicle_velocity_measured_ = measured_velocity;
        if (Math.abs(vehicle_velocity_measured_.dtheta) < 2.0 * Math.PI) {
            // Reject really high angular velocities from the filter.
            vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        } else {
            vehicle_velocity_measured_filtered_
                    .add(new Twist2d(vehicle_velocity_measured_.dx, vehicle_velocity_measured_.dy, 0.0));
        }
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized void resetVision() {
        vision_target_.reset();
    }
    

    private Translation2d getCameraToVisionTargetPose(double timestamp, TargetInfo target, Limelight source) {
        // Compensate for camera pitch
        final Rotation2d limelight_angle = getVehicleToHood(timestamp);
        final Rotation2d hood_angle = Rotation2d
                .fromDegrees(90 - limelight_angle.getDegrees() - Constants.kLimelightPitchOffset);
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(limelight_angle);
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double lens_height = source.getLensHeight() + (Constants.kHoodRadius * hood_angle.sin());
        double differential_height = lens_height - (Constants.kGoalHeight);
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }

        return null;
    }

    private void updateGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPoses, GoalTracker tracker,
            Limelight source) {
        if (cameraToVisionTargetPoses.size() != 1
                || cameraToVisionTargetPoses.get(0) == null /*
                                                             * || cameraToVisionTargetPoses.get(1) == null
                                                             */)
            return;
        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetPoses.get(0));

        Pose2d fieldToVisionTarget = getFieldToTurret(timestamp)
                .transformBy(Pose2d.fromTranslation(getTurretToLens(timestamp))).transformBy(cameraToVisionTarget);
        tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), new Rotation2d())));
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations) {
        mCameraToVisionTargetPoses.clear();

        if (observations == null || observations.isEmpty()) {
            vision_target_.update(timestamp, new ArrayList<>());
            return;
        }

        Limelight source = Limelight.getInstance();

        for (TargetInfo target : observations) {
            mCameraToVisionTargetPoses.add(getCameraToVisionTargetPose(timestamp, target, source));
        }

        updateGoalTracker(timestamp, mCameraToVisionTargetPoses, vision_target_, source);
    }

    // use known field target orientations to compensate for inaccuracy, assumes
    // robot starts pointing directly away
    // from and perpendicular to alliance wall
    private final double[] kPossibleTargetNormals = { 0.0, 90.0, 180.0, 270.0 };

    public synchronized Pose2d getFieldToVisionTarget() {
        GoalTracker tracker = vision_target_;

        if (!tracker.hasTracks()) {
            return null;
        }

        Pose2d fieldToTarget = tracker.getTracks().get(0).field_to_target;

        double normalPositive = (fieldToTarget.getRotation().getDegrees() + 360) % 360;
        double normalClamped = kPossibleTargetNormals[0];
        for (double possible : kPossibleTargetNormals) {
            if (Math.abs(normalPositive - possible) < Math.abs(normalPositive - normalClamped)) {
                normalClamped = possible;
            }
        }

        return new Pose2d(fieldToTarget.getTranslation(), Rotation2d.fromDegrees(normalClamped));
    }

    public synchronized Pose2d getVehicleToVisionTarget(double timestamp) {
        Pose2d fieldToVisionTarget = getFieldToVisionTarget();

        if (fieldToVisionTarget == null) {
            return null;
        }

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget);
    }

    public synchronized Optional<AimingParameters> getAimingParameters(boolean inner_goal, int prev_track_id,
            double max_track_age) {
        GoalTracker tracker = vision_target_;
        List<GoalTracker.TrackReport> reports = tracker.getTracks();

        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double timestamp = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = new TrackReportComparator(Constants.kTrackStabilityWeight,
                Constants.kTrackAgeWeight, Constants.kTrackSwitchingWeight, prev_track_id, timestamp);
        reports.sort(comparator);

        GoalTracker.TrackReport report = null;
        for (GoalTracker.TrackReport track : reports) {
            if (track.latest_timestamp > timestamp - max_track_age) {
                report = track;
                break;
            }
        }
        if (report == null) {
            return Optional.empty();
        }
        Pose2d latestTurretFixedToField = getPredictedFieldToVehicle(Constants.kAutoAimPredictionTime)
                .transformBy(kVehicleToTurretFixed).inverse();
        Pose2d latestTurretFixedToGoal = latestTurretFixedToField.transformBy(report.field_to_target);

        Pose2d vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(report.field_to_target)
                .transformBy(getVisionTargetToGoalOffset(inner_goal));

        AimingParameters params = new AimingParameters(latestTurretFixedToGoal, report.field_to_target,
                report.field_to_target.getRotation(), report.latest_timestamp, report.stability, report.id);
        return Optional.of(params);
    }

    public Pose2d getRobot() {
        return new Pose2d();
    }

    public synchronized Pose2d getVisionTargetToGoalOffset(boolean inner_goal) {
        if (inner_goal) {
            return Pose2d.fromTranslation(new Translation2d(Constants.kInnerGoalDepth, 0));
        } else {
            return Pose2d.fromTranslation(new Translation2d(0, 0));
        }
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());
        SmartDashboard.putString("Robot Field to Vehicle", getLatestFieldToVehicle().getValue().toString());
        SmartDashboard.putNumber("Robot X", getLatestFieldToVehicle().getValue().getTranslation().x());
        SmartDashboard.putNumber("Robot Y", getLatestFieldToVehicle().getValue().getTranslation().y());
        SmartDashboard.putNumber("Robot Theta", getLatestFieldToVehicle().getValue().getRotation().getDegrees());
        SmartDashboard.putNumber("Limelight Pitch", getVehicleToHood(Timer.getFPGATimestamp()).getDegrees());
        Optional<AimingParameters> params = getAimingParameters(false, -1, Constants.kMaxGoalTrackAge);
        SmartDashboard.putBoolean("Has Aiming Parameters", params.isPresent());
        if (params.isPresent()) {
            SmartDashboard.putNumber("Vehicle to Target", params.get().getRange());
            SmartDashboard.putNumber("Vehicle to TargetAngle", params.get().getTurretToGoalRotation().getDegrees());

        }
    }
}