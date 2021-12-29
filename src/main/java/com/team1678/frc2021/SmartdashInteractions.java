package com.team1678.frc2021;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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
    private ShuffleboardTab PID_TAB;

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
    private final NetworkTableEntry[] mSwerveIntegrated = new NetworkTableEntry[4];
    private final NetworkTableEntry[] mSwerveDrivePercent = new NetworkTableEntry[4];
    private final NetworkTableEntry mSwerveOdometryX;
    private final NetworkTableEntry mSwerveOdometryY;
    private final NetworkTableEntry mSwerveOdometryRot;
    private final NetworkTableEntry mPIDEnableToggle;
    private final NetworkTableEntry mDesiredAngleP;
    private final NetworkTableEntry mDesiredAngleI;
    private final NetworkTableEntry mDesiredAngleD;
    private final NetworkTableEntry mCurrentAngleP;
    private final NetworkTableEntry mCurrentAngleI;
    private final NetworkTableEntry mCurrentAngleD;
    private final NetworkTableEntry[] mModuleAngleCurrent = new NetworkTableEntry[4];
    private final NetworkTableEntry[] mModuleAngleGoals = new NetworkTableEntry[4];

    public SmartdashInteractions() {
        /* Get Subsystems */
        mLimelight = Limelight.getInstance();
        mSuperstructure = Superstructure.getInstance();
        mSwerve = Swerve.getInstance();
        mSwerveModules = Swerve.getInstance().mSwerveMods;

        /* Get Tabs */
        VISION_TAB = Shuffleboard.getTab("Vision");
        SWERVE_TAB = Shuffleboard.getTab("Swerve");
        PID_TAB = Shuffleboard.getTab("Module PID");
        
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
            mSwerveIntegrated[i] = mSwerveAngles[i].add("Integrated", 0.0)
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

            mModuleAngleCurrent[i] = PID_TAB.add("Module " + i + " Current", 0.0)
                .withPosition(i, 2)
                .withSize(1, 1)
                .getEntry();

            mModuleAngleGoals[i] = PID_TAB.add("Module " + i + " Target", 0.0)
                .withPosition(i, 3)
                .withSize(1, 1)
                .getEntry();
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

        mPIDEnableToggle = PID_TAB
            .add("Apply PID", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(3, 0)
            .withSize(2, 1)
            .getEntry();

        TalonFXConfiguration currentAngleValues = CTREConfigs.swerveAngleFXConfig();

        mDesiredAngleP = PID_TAB
            .add("Wanted P", currentAngleValues.slot0.kP)
            .withPosition(0,0)
            .withSize(1, 1)
            .getEntry();

        mDesiredAngleI = PID_TAB
            .add("Wanted I", currentAngleValues.slot0.kI)
            .withPosition(1,0)
            .withSize(1, 1)
            .getEntry();

        mDesiredAngleD = PID_TAB
            .add("Wanted D", currentAngleValues.slot0.kD)
            .withPosition(2,0)
            .withSize(1, 1)
            .getEntry();

        mCurrentAngleP = PID_TAB
            .add("Current P", 0.0)
            .withPosition(0,1)
            .withSize(1, 1)
            .getEntry();

        mCurrentAngleI = PID_TAB
            .add("Current I", 0.0)
            .withPosition(1,1)
            .withSize(1, 1)
            .getEntry();

        mCurrentAngleD = PID_TAB
            .add("Current D", 0.0)
            .withPosition(2,1)
            .withSize(1, 1)
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
            mSwerveCancoders[i].setDouble(truncate(mSwerveModules[i].getCanCoder().getDegrees()));
            mSwerveIntegrated[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            mSwerveDrivePercent[i].setDouble(truncate(mSwerveModules[i].getState().speedMetersPerSecond));

            mModuleAngleCurrent[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            mModuleAngleGoals[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getTargetAngle(), 0, 360)));

        }
        mSwerveOdometryX.setDouble(truncate(mSwerve.getPose().getX()));
        mSwerveOdometryY.setDouble(truncate(mSwerve.getPose().getY()));
        mSwerveOdometryRot.setDouble(truncate(mSwerve.getPose().getRotation().getDegrees()));

        if(mPIDEnableToggle.getValue().getBoolean()) {
            mSwerve.setAnglePIDValues(mDesiredAngleP.getValue().getDouble(), mDesiredAngleI.getValue().getDouble(), mDesiredAngleD.getValue().getDouble());
        }
        double[] currentPIDVals = mSwerve.getAnglePIDValues(0);
        mCurrentAngleP.setDouble(currentPIDVals[0]);
        mCurrentAngleI.setDouble(currentPIDVals[1]);
        mCurrentAngleD.setDouble(currentPIDVals[2]);
    }

    private double truncate(double number) {
        return Math.floor(number * 100) / 100;
    }
}
 