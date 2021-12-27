package com.team1678.frc2021;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);

        config.slot0.kP = Constants.Swerve.driveKP;
        config.slot0.kI = Constants.Swerve.driveKI;
        config.slot0.kD = Constants.Swerve.driveKD;
        config.slot0.kF = Constants.Swerve.driveKF;        
        config.supplyCurrLimit = driveSupplyLimit;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.openloopRamp = Constants.Swerve.openLoopRamp;
        config.closedloopRamp = Constants.Swerve.closedLoopRamp;
        return config;
    }

    public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit, 
            Constants.Swerve.angleContinuousCurrentLimit, 
            Constants.Swerve.anglePeakCurrentLimit, 
            Constants.Swerve.anglePeakCurrentDuration);

        angleConfig.slot0.kP = Constants.Swerve.angleKP;
        angleConfig.slot0.kI = Constants.Swerve.angleKI;
        angleConfig.slot0.kD = Constants.Swerve.angleKD;
        angleConfig.slot0.kF = Constants.Swerve.angleKF;
        angleConfig.supplyCurrLimit = angleSupplyLimit;
        angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return angleConfig;
    }

    public static CANCoderConfiguration swerveCancoderConfig() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = Constants.Swerve.canCoderInvert;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        return config;
    }
}
