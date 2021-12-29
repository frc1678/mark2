package com.team1678.frc2021.states;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.PolynomialRegression;



public class SuperstructureConstants {
    public static final double kTurretPaddingDegrees = 3;
    public static final double kHoodPaddingDegrees = 2;
    public static final double kShooterPaddingVelocity = 100;


    public static final double[] kPadding = {
            kTurretPaddingDegrees, kShooterPaddingVelocity, kHoodPaddingDegrees};

    //turret
    public static final double kTurretDOF = 360;


    //hood
    public static double kDefaultHoodAngle = Math.toRadians(0);
    public static boolean kUseHoodAutoAimPolynomial = false;

    public static boolean kUseSmartdashboard = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kHoodAutoAimPolynomial;

    public static double[][] kHoodManualAngle = {
        // Shop Field Regression
        // { 25.6, 35 }, // good
        // { 42.1, 43 }, // good
        // { 64.8, 57 }, // good
        // { 84.7, 63 }, // good
        // { 104.0, 60 }, // good
        // { 118.0, 73 }, // good
        // { 140.6, 76 }, // good
        // { 149.4, 77 }, // good 
        // { 178.0, 79 },
        // { 199.0, 80 },
        // { 215, 81 }

        // Chezy Regression
        { 25.6, 35 },
        { 31.0, 40 },
        { 42.1, 43 },
        { 56.0, 53 },
        { 64.8, 57 },
        { 84.0, 64 },
        { 111.0, 70 },
        { 131.0, 74 },
        { 154.0, 75 }

    };

    static {
        //iterate through the array and place each point into the interpolating tree
        for (double[] pair : kHoodManualAngle) {
            kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        
        kHoodAutoAimPolynomial = new PolynomialRegression(kHoodManualAngle, 1);
    }
    
    //shooter
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kFlywheelAutoAimPolynomial;

    public static double[][] kFlywheelManualRPM = {
        // Shop Field Regression
        // { 25.6, 1900 },
        // { 64.8, 2200 },
        // { 84.7, 2500 },
        // { 104.0, 2800 },
        // { 118.5, 3100 },
        // { 140.6, 3200 }, 
        // { 149.6, 3500 }, 
        // { 178.0, 3800 },
        // { 199.0, 3900 },
        // { 215, 4000 },

        // Chezy Regression
        { 25.6, 1900 },
        { 31.0, 1900 },
        { 56.0, 2200 },
        { 64.8, 2200 },
        { 84.0, 2500 },
        { 111.0, 3000 },
        { 131.0, 3100 },
        { 154.0, 3400 }
    };

    static {
        for (double[] pair : kFlywheelManualRPM) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelManualRPM, 2);
    }

}
