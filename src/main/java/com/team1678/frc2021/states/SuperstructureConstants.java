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

    public static boolean kUseSmartdashboard = true;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kHoodAutoAimPolynomial;

    public static double[][] kHoodManualAngle = {
        // { 49.0, 52.0},
        // { 66.0, 60.},
        // { 84.7, 70.},
        // { 102.5, 75.0 },
        // { 112.7, 77.0 },
        // { 126.6, 78.0 },
        // { 140.4, 81.5 },
        // { 150.1, 82.0 },
        // { 160.3, 83.0 },
        // { 173.0, 83.3 },

        // { 22.2, 31 },
        // { 51.3, 55 },
        // { 79.0, 67 },
        // { 100.0, 70 },
        // { 115.4, 74 },
        // { 125.0, 77 },
        // { 136.3, 80 },
        // { 149.4, 79 },
        // { 167.8, 82 },

        { 25.6, 35 }, // good
        { 42.1, 43 }, // good
        { 64.8, 57 }, // good
        { 84.7, 63 }, // good
        { 104.0, 60 }, // good
        { 118.0, 73 }, // good
        { 140.6, 76 }, // good
        { 149.4, 77 }, // good 
        { 178.0, 79 },
        { 199.0, 80 },
        { 215, 81 }
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
        // { 49.0, 1800},
        // { 66.0, 2300},
        // { 84.7, 2700},
        // { 102.5, 3400 },
        // { 112.7, 3500 },
        // { 126.6, 3800 },
        // { 140.4, 4000 },
        // { 150.1, 4000 },
        // { 160.3, 4000 },
        // { 173.0, 4000 },

        // { 22.2, 2300 },
        // { 51.3, 2500 },
        // { 79.0, 2600 },
        // { 100.0, 2700 },
        // { 115.4, 3000 },
        // { 125.0, 3200 },
        // { 136.3, 3300 },
        // { 149.4, 3600 },
        // { 167.8, 4000 },

        { 25.6, 1700 }, // good
        { 64.8, 2000 }, // good
        { 79.0, 2600 }, // good
        { 84.7, 2300 }, // good
        { 104.0, 2600 }, // good
        { 118.5, 2900 }, // good
        { 140.6, 3000 }, // good
        { 149.6, 3300 }, // good
        { 178.0, 3600 },
        { 199.0, 3700 },
        { 215, 4000 },

        // TODO: Fill in with values
    };

    static {
        for (double[] pair : kFlywheelManualRPM) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelManualRPM, 2);
    }

}
