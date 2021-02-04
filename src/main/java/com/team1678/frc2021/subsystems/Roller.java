package com.team1678.frc2021.subsystems;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper.ColorSensorData;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

// Control panel manipulator
public class Roller extends Subsystem {
    // Constants
    public static double kRotateVoltage = -6.0; // Positive value rotates the control panel counter-clockwise

    // Motors, solenoids and sensors
    //public I2C.Port i2cPort;
    //public ColorSensorV3 mColorSensor;
    public REVColorSensorV3Wrapper mColorSensor;
    private final PWMSparkMax mRollerMotor;
    public Solenoid mPopoutSolenoid;

    // Color sensing
    private final ColorMatch mColorMatcher = new ColorMatch();
    ColorMatchResult mMatch;

    private final Color kBlueTarget = ColorMatch.makeColor(0.134, 0.432, 0.434);
    private final Color kGreenTarget = ColorMatch.makeColor(0.178, 0.571, 0.251);
    private final Color kRedTarget = ColorMatch.makeColor(0.485, 0.364, 0.150);
    private final Color kYellowTarget = ColorMatch.makeColor(0.314, 0.553, 0.120);

    private int mColorCounter = 0;

    private Color mInitialColor;
    private Color mOneColorAgo;
    private Color mTwoColorsAgo;

    private Color mColorPositionTarget;
    private Color mSlowDownTarget;
    private boolean mRunningManual = false;

    // Game data
    private String gameData;
    private String colorString = "Unknown";
    private boolean mSolenoidOut = false;
    private TimeDelayedBoolean mSolenoidTimer = new TimeDelayedBoolean();
    private TimeDelayedBoolean mPositionalTimer = new TimeDelayedBoolean();

    // State management
    public enum WantedAction {
        NONE, ACHIEVE_ROTATION_CONTROL, ACHIEVE_POSITION_CONTROL, SOLENOID_OUT_ONLY,
    }

    private enum State {
        IDLE, ACHIEVING_ROTATION_CONTROL, ACHIEVING_POSITION_CONTROL, SOLENOID_OUT, FINISHED,
    }

    private State mState = State.IDLE;

    // General management
    private static Roller mInstance;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private Roller() {
        mRollerMotor = new PWMSparkMax(Constants.kRollerId);
        mPopoutSolenoid = Constants.makeSolenoidForId(Constants.kRollerSolenoid);

        //i2cPort = I2C.Port.kOnboard;
        mColorSensor = new REVColorSensorV3Wrapper(I2C.Port.kOnboard);

        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget); 
        
        mColorSensor.start();
    }

    public synchronized static Roller getInstance() {
        if (mInstance == null) {
            mInstance = new Roller();
        }

        return mInstance;
    }

    public synchronized void setGameData(String data) {
        gameData = data;

        if(gameData.length() > 0) {
            // Accounts for the FMS detecting the color two wedges down
            switch (gameData.charAt(0)) {
                case 'B' :
                    mColorPositionTarget = kRedTarget;
                    mSlowDownTarget = kGreenTarget;
                    break;
                case 'G' :
                    mColorPositionTarget = kYellowTarget;
                    mSlowDownTarget = kRedTarget;
                    break;
                case 'R' :
                    mColorPositionTarget = kBlueTarget;
                    mSlowDownTarget = kYellowTarget;
                    break;
                case 'Y' :
                    mColorPositionTarget = kGreenTarget;
                    mSlowDownTarget = kBlueTarget;
                    break;
                default :
                    mColorPositionTarget = kRedTarget;
                    mSlowDownTarget = kGreenTarget;
                    System.out.println("Invalid color from FMS!");
                    break;
            }
        } else {
            // No data has been recieved
        }
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public synchronized void readPeriodicInputs() {
        ColorSensorData reading = mColorSensor.getLatestReading();
        mPeriodicIO.timestamp = reading.timestamp;
        mPeriodicIO.distance = reading.distance;
        
        if (reading.color != null) {
            mPeriodicIO.detected_color = reading.color;
            mMatch = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);
        }

        if (mMatch != null) {
            if (mMatch.color == kBlueTarget) {
                colorString = "Blue";
            } else if (mMatch.color == kRedTarget) {
                colorString = "Red";
            } else if (mMatch.color == kGreenTarget) {
                colorString = "Green";
            } else if (mMatch.color == kYellowTarget) {
                colorString = "Yellow";
            } else {
                colorString = "Unknown";
            }
        } else {
            colorString = "Unknown";
        }

        boolean solenoid_ret = mSolenoidTimer.update(mPeriodicIO.pop_out_solenoid, 0.5);
        if (solenoid_ret == true && !mSolenoidOut) {
            mInitialColor = mOneColorAgo = mTwoColorsAgo = mMatch.color;
        }
        mSolenoidOut = solenoid_ret;
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public synchronized void writePeriodicOutputs() {
        mPopoutSolenoid.set(mPeriodicIO.pop_out_solenoid);
        if (mSolenoidOut) {
            mRollerMotor.set(mPeriodicIO.roller_demand / 12.0);
        } else {
            mRollerMotor.set(0);
        }
        //mRollerMotor.set(0.1);
        //mPopoutSolenoid.set(true);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
                mColorSensor.start();
            } 

            @Override 
            public void onLoop(double timestamp) {
                synchronized (Roller.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                mColorSensor.stop();
            }

        });
    }

    public void runManual(double voltage) {
        mRunningManual = true;
        mPeriodicIO.pop_out_solenoid = true;
        mPeriodicIO.roller_demand = voltage;
    }

    // TODO: Only perform actions if 'modifyOutputs' is true
    public void runStateMachine() {
        if (mRunningManual) {
            return;
        }
        switch(mState) {
            case IDLE:
                mPeriodicIO.roller_demand = 0.0;
                mPeriodicIO.pop_out_solenoid = false;
                break;
            case ACHIEVING_ROTATION_CONTROL:
                mPeriodicIO.pop_out_solenoid = true;

                if (!mSolenoidOut) {
                    return;
                }

                if (mColorCounter < 9) {
                    if (colorString != "Unknown") {
                        mPeriodicIO.roller_demand = kRotateVoltage;
                
                        if (mMatch.color != mOneColorAgo) {
                            if (mMatch.color == mInitialColor) {
                                // Necessary to avoid color confusion between red/green and blue/yellow
                                // Haven't tested this logic yet
                                if (!(mInitialColor == kYellowTarget && mMatch.color == kYellowTarget && mOneColorAgo == kGreenTarget && mTwoColorsAgo == kBlueTarget)
                                || !(mInitialColor == kGreenTarget && mMatch.color == kGreenTarget && mOneColorAgo == kYellowTarget && mTwoColorsAgo == kRedTarget)) {
                                    mColorCounter++;
                                }
                            }
                        }
                    }
                  }
            
                  if (mColorCounter >= 7) {
                    mColorCounter = 0;
                    setState(WantedAction.SOLENOID_OUT_ONLY);
                  }

                  if (mOneColorAgo != mMatch.color) {
                    mTwoColorsAgo = mOneColorAgo;
                    mOneColorAgo = mMatch.color;
                  }

                break;
            case ACHIEVING_POSITION_CONTROL:
                if (gameData.length() > 0) {
                    mPeriodicIO.pop_out_solenoid = true;

                    if (!mSolenoidOut) {
                        return;
                    }

                        if (mMatch.color != mColorPositionTarget || (!mPositionalTimer.update(mMatch.color == mColorPositionTarget, .8) && mMatch.color == mColorPositionTarget)) {
                            double adjustedRotateVoltage = kRotateVoltage;
                            double adjustedSlowRotateVoltage = kRotateVoltage * 0.25;
                            
                            /*if ((mInitialColor == kRedTarget && mColorPositionTarget == kGreenTarget) || 
                            (mInitialColor == kGreenTarget && mColorPositionTarget == kBlueTarget) ||
                            (mInitialColor == kYellowTarget && mColorPositionTarget == kRedTarget) ||
                            (mInitialColor == kBlueTarget && mColorPositionTarget == kYellowTarget)) {
                                adjustedRotateVoltage = -adjustedRotateVoltage;
                                adjustedSlowRotateVoltage = -adjustedSlowRotateVoltage;
                            }*/
                
                            //if (mMatch.color == mSlowDownTarget) {
                                mPeriodicIO.roller_demand = adjustedSlowRotateVoltage;
                            //} else {
                            //    mPeriodicIO.roller_demand = adjustedRotateVoltage;
                            //}
                        } else {
                            mColorCounter = 0;
                            setState(WantedAction.SOLENOID_OUT_ONLY);
                        }
                }

                break;
            case FINISHED:
            case SOLENOID_OUT:
                mPeriodicIO.roller_demand = 0;
                mPeriodicIO.pop_out_solenoid = true;
                break;
            default:
                System.out.println("Invalid roller goal!");
                break;
        }
    }

    public void zeroSensors() {}

    @Override
    public void stop() {
        mPeriodicIO.roller_demand = 0.0;
        mPeriodicIO.pop_out_solenoid = false;
        //setState(WantedAction.NONE);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("Color", colorString);

        SmartDashboard.putNumber("Color Data t", mPeriodicIO.timestamp);
        SmartDashboard.putNumber("Color Data distance", mPeriodicIO.distance);

        
        SmartDashboard.putNumber("Color Counter", mColorCounter);
        SmartDashboard.putString("Roller State", mState.toString());
        SmartDashboard.putNumber("Roller Demand", mPeriodicIO.roller_demand);
        SmartDashboard.putBoolean("Roller Out", mPeriodicIO.pop_out_solenoid);
        SmartDashboard.putString("Game Data", gameData);

        mColorSensor.outputToSmartDashboard();
    }

    public void setState(WantedAction action) {
        mRunningManual = false;
        switch(action) {
            case NONE:
                break;
            case ACHIEVE_ROTATION_CONTROL:
                // This is frowned upon by Java developers, so I'm willing to change it
                mInitialColor = mOneColorAgo = mTwoColorsAgo = mMatch.color;
                if (!mPeriodicIO.pop_out_solenoid) {
                    mState = State.SOLENOID_OUT;
                } else {
                    if (mState != State.FINISHED && mState != State.ACHIEVING_ROTATION_CONTROL) {
                        mState = State.ACHIEVING_ROTATION_CONTROL;
                    } else {
                        mState = State.IDLE;
                    }
                }
                break;
            case ACHIEVE_POSITION_CONTROL:
                if (!mPeriodicIO.pop_out_solenoid) {
                    mState = State.SOLENOID_OUT;
                } else {
                    if (mState != State.FINISHED && mState != State.ACHIEVING_POSITION_CONTROL) {
                        mState = State.ACHIEVING_POSITION_CONTROL;
                    } else {
                        mState = State.IDLE;
                    }
                }
                break;
            case SOLENOID_OUT_ONLY:
                mState = State.SOLENOID_OUT;
                break;
            default:
                System.out.println("Invalid roller action!");
                break;
        }
    }

    public static class PeriodicIO {
        // INPUTS
        private double timestamp;

        public Color detected_color;
        private int distance;

        // OUTPUTS
        public double roller_demand;
        public boolean pop_out_solenoid;
    }
}