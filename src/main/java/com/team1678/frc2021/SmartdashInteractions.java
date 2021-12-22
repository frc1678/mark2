package com.team1678.frc2021;

import com.team1678.frc2021.subsystems.Limelight;
import com.team1678.frc2021.subsystems.Superstructure;
import com.team1678.frc2021.subsystems.Swerve;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpiutil.math.MathUtil;

public class SmartdashInteractions {

    /* SmartdashInteractions Instance */
    private static SmartdashInteractions mInstance; 

    public static SmartdashInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new SmartdashInteractions();
        }
        return mInstance;
    }

    /* Subsystem Dependencies */
    private final Limelight mLimelight;
    private final Superstructure mSuperstructure;
    private final Swerve mSwerve;
    private final SwerveModule[] mSwerveModules;

    /* Tabs */
    private ShuffleboardTab VISION_TAB;
    private ShuffleboardTab SWERVE_TAB;
    //private ShuffleboardTab ANGLES_TAB;

    /* Entries */

    /* Vision */
    private final NetworkTableEntry mSeesTarget;
    private final NetworkTableEntry mLimelightOK;
    private final NetworkTableEntry mLimelightLatency;
    private final NetworkTableEntry mLimelightDT;
    private final NetworkTableEntry mLimelightTX;
    private final NetworkTableEntry mLimelightTY;

    /* Superstructure */
    private final NetworkTableEntry mTurretMode;
    private final NetworkTableEntry mOnTarget;

    /* Swerve Modules */
    private final String[] kSwervePlacements = {"Front Left", "Front Right", "Back Left", "Back Right"};
    private final ShuffleboardLayout[] mSwerveAngles = new ShuffleboardLayout[4];
    private final NetworkTableEntry[] mSwerveCancoders = new NetworkTableEntry[4];
    private final NetworkTableEntry[] mSwerveAdjustedAngle = new NetworkTableEntry[4];
    private final NetworkTableEntry[] mSwerveDrivePercent = new NetworkTableEntry[4];
    //private final NetworkTableEntry[] mSwerveModuleAngleDials = new NetworkTableEntry[4];
    private final NetworkTableEntry mSwerveOdometryX;
    private final NetworkTableEntry mSwerveOdometryY;
    private final NetworkTableEntry mSwerveOdometryRot;

    public SmartdashInteractions() {
        /* Get Subsystems */
        mLimelight = Limelight.getInstance();
        mSuperstructure = Superstructure.getInstance();
        mSwerve = Swerve.getInstance();
        mSwerveModules = Swerve.getInstance().mSwerveMods;

        /* Get Tabs */
        VISION_TAB = Shuffleboard.getTab("Vision");
        SWERVE_TAB = Shuffleboard.getTab("Swerve");
        //ANGLES_TAB = Shuffleboard.getTab("Module Angles");
        
        /* Create Entries */
        mLimelightOK = VISION_TAB
            .add("Limelight OK", false)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();        
        mSeesTarget = VISION_TAB
            .add("Limelight Sees Target", false)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();
        mLimelightLatency = VISION_TAB
            .add("Limelight Latency", -1.0)
            .withPosition(2, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        mLimelightDT = VISION_TAB
            .add("Limelight Loop Time", -1.0)
            .withPosition(4, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        mLimelightTX = VISION_TAB
            .add("Limelight TX", 0.0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();
        mLimelightTY = VISION_TAB
            .add("Limelight TY", 0.0)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();    
        mTurretMode = VISION_TAB
            .add("Turret Mode", "N/A")
            .withPosition(1, 2)
            .withSize(1, 1)
            .getEntry(); 
        mOnTarget = VISION_TAB
            .add("On Target", false)
            .withPosition(0, 2)
            .withSize(1, 1)
            .getEntry();

        for (int i = 0; i < mSwerveCancoders.length; i++) {
            mSwerveAngles[i] = SWERVE_TAB
                .getLayout("Module " + i + " Angle", BuiltInLayouts.kGrid)
                .withSize(2, 2)
                .withPosition(i * 2, 0);

            mSwerveCancoders[i] = mSwerveAngles[i].add("Cancoder", 0.0)
                .withPosition(0, 0)
                .withSize(5, 1)
                .getEntry();
            mSwerveCancoders[i] = mSwerveAngles[i].add("Location", kSwervePlacements[i])
                .withPosition(1, 0)
                .withSize(5, 1)
                .getEntry();
            mSwerveAdjustedAngle[i] = mSwerveAngles[i].add("Integrated", 0.0)
                .withPosition(0, 1)
                .withSize(5, 1)
                .getEntry();
            mSwerveAngles[i].add("Offset", mSwerve.mSwerveMods[i].angleOffset)
                .withPosition(0, 2)
                .withSize(5, 1)
                .getEntry();

            mSwerveDrivePercent[i] = SWERVE_TAB
                .add("Swerve Module " + i + " MPS ", 0.0)
                .withPosition(i * 2, 2)
                .withSize(2, 1)
                .getEntry();

            // mSwerveModuleAngleDials[i] = ANGLES_TAB
            //     .add("Swerve Module " + i + " Angle", 0.0)
            //     .withWidget(BuiltInWidgets.kGyro)
            //     .withPosition(i * 2, 0)
            //     .withSize(2, 2)
            //     .getEntry();
        }

        mSwerveOdometryX = SWERVE_TAB
            .add("Odometry X", 0)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();

        mSwerveOdometryY = SWERVE_TAB
            .add("Odometry Y", 0)
            .withPosition(2, 3)
            .withSize(2, 1)
            .getEntry();

        mSwerveOdometryRot = SWERVE_TAB
            .add("Pigeon Angle", 0)
            .withPosition(4, 3)
            .withSize(2, 1)
            .getEntry();
    }

    public void update() {
        
        /* Vision */
        mSeesTarget.setBoolean(mLimelight.seesTarget());
        mLimelightOK.setBoolean(mLimelight.limelightOK());
        mLimelightLatency.setDouble(mLimelight.getLatency());
        mLimelightDT.setDouble(mLimelight.getDt());
        mLimelightTX.setDouble(mLimelight.getOffset()[0]);
        mLimelightTY.setDouble(mLimelight.getOffset()[1]);

        /* Superstructure */
        mOnTarget.setBoolean(mSuperstructure.isAimed());
        mTurretMode.setString(mSuperstructure.getTurretControlMode().toString());
        
        /* Swerve */
        for (int i = 0; i < mSwerveCancoders.length; i++) {
            mSwerveCancoders[i].setDouble(truncate(Math.floor(mSwerveModules[i].getCanCoder().getDegrees()* 100) / 100));
            mSwerveAdjustedAngle[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            //mSwerveModuleAngleDials[i].forceSetDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            mSwerveDrivePercent[i].setDouble(truncate(mSwerveModules[i].getState().speedMetersPerSecond));
        }
        mSwerveOdometryX.setDouble(truncate(mSwerve.getPose().getX()));
        mSwerveOdometryY.setDouble(truncate(mSwerve.getPose().getY()));
        mSwerveOdometryRot.setDouble(truncate(mSwerve.getPose().getRotation().getDegrees()));
    }

    private double truncate(double number) {
        return Math.floor(number * 100) / 100;
    }
}
 