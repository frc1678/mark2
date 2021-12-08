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
    private final Limelight kLimelight;
    private final Superstructure kSuperstructure;

    /* Tabs */
    private final ShuffleboardTab kVisionTab;

    /* Entries */

    /* Vision */
    private final NetworkTableEntry kSeesTarget;
    private final NetworkTableEntry kLimelightOK;
    private final NetworkTableEntry kLimelightLatency;
    private final NetworkTableEntry kLimelightDT;
    private final NetworkTableEntry kLimelightTX;
    private final NetworkTableEntry kLimelightTY;

    /* Superstructure */
    private final NetworkTableEntry kTurretMode;
    private final NetworkTableEntry kOnTarget;


    public SmartdashInteractions() {
        /* Get Subsystems */
        kLimelight = Limelight.getInstance();
        kSuperstructure = Superstructure.getInstance();

        /* Get Tabs */
        kVisionTab = Shuffleboard.getTab("Vision");
        
        /* Create Entries */
        kLimelightOK = kVisionTab
            .add("Limelight OK", false)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();        
        kSeesTarget = kVisionTab
            .add("Limelight Sees Target", false)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();
        kLimelightLatency = kVisionTab
            .add("Limelight Latency", -1.0)
            .withPosition(2, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
        kLimelightDT = kVisionTab
            .add("Limelight Loop Time", -1.0)
            .withPosition(4, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
        kLimelightTX = kVisionTab
            .add("Limelight TX", 0.0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();
        kLimelightTY = kVisionTab
            .add("Limelight TY", 0.0)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();    
        kTurretMode = kVisionTab
            .add("Turret Mode", "N/A")
            .withPosition(1, 2)
            .withSize(1, 1)
            .getEntry(); 
        kOnTarget = kVisionTab
            .add("On Target", false)
            .withPosition(0, 2)
            .withSize(1, 1)
            .getEntry();    
    }

    public void update() {
        /* Vision */
        kSeesTarget.setBoolean(kLimelight.seesTarget());
        kLimelightOK.setBoolean(kLimelight.limelightOK());
        kLimelightLatency.setDouble(kLimelight.getLatency());
        kLimelightDT.setDouble(kLimelight.getDT());
        kLimelightTX.setDouble(kLimelight.getOffset()[0]);
        kLimelightTY.setDouble(kLimelight.getOffset()[1]);

        /* Superstructure */
        kOnTarget.setBoolean(kSuperstructure.isAimed());
        kTurretMode.setString(kSuperstructure.getTurretControlMode().toString());
        
    }
}
 