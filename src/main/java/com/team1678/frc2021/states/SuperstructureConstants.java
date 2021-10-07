package com.team1678.frc2021.states;

import com.team254.lib.util.PolynomialRegression;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.InterpolatingDouble;



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
        { 49.0, 52.0},
        { 66.0, 60.},
        { 84.7, 70.},
        { 102.5, 75.0 },
        { 112.7, 77.0 },
        { 127.0, 78.0 },
        { 142.7, 80.0 },
        { 152.7, 81.0 },
        // { 257.0, 83.0 },
        // //{ 318.0, 89.0 },
        // { 330.0, 88.5 },
        
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
        { 49.0, 1800},
        { 66.0, 2300},
        { 84.7, 2700},
        { 102.5, 3400 },
        { 112.7, 3500 },
        { 127.0, 3900 },
        { 142.7, 4000 },
        { 152.7, 4000 },
        // { 265.0, 3300 },
        // { 318.0, 3600 },
        // { 330.0, 3700 },
        
        // TODO: Fill in with values
    };

    static {
        for (double[] pair : kFlywheelManualRPM) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelManualRPM, 2);
    }

}
