package com.team1323.lib.vision;

import com.team254.lib.geometry.Rotation2d;

/**
 * A container class to specify the shooter angle. It contains the desired range, the field_to_goal_angle
 */
public class ShooterAimingParameters {
    double range;
    double last_seen_timestamp;
    double stability;
    Rotation2d turret_angle;

    public ShooterAimingParameters(double range, Rotation2d turret_angle, double last_seen_timestamp,
            double stability) {
        this.range = range;
        this.turret_angle = turret_angle;
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getTurretAngle() {
        return turret_angle;
    }

    public double getLastSeenTimestamp() {
        return last_seen_timestamp;
    }

    public double getStability() {
        return stability;
    }

}
