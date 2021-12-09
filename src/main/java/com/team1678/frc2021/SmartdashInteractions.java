package com.team1678.frc2021;

import com.team1678.frc2021.subsystems.Limelight;
import com.team1678.frc2021.subsystems.Superstructure;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

    /* Tabs */
    private ShuffleboardTab VISION_TAB;

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


    public SmartdashInteractions() {
        /* Get Subsystems */
        mLimelight = Limelight.getInstance();
        mSuperstructure = Superstructure.getInstance();

        /* Get Tabs */
        VISION_TAB = Shuffleboard.getTab("Vision");
        
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
        
    }
}
 