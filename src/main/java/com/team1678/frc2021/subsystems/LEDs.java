/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.team1323.lib.util.HSVtoRGB;
import com.team1323.lib.util.MovingAverage;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shiny Lights
 */
public class LEDs extends Subsystem{
    private static LEDs instance = null;
    public static LEDs getInstance(){
        if(instance == null)
            instance = new LEDs();
        return instance;
    }

    CANifier canifier;

    public LEDs(){
        canifier = Canifier.getInstance().getCanifier();
    }

    boolean lit = false;
    double lastOnTime = 0.0;
    double lastOffTime = 0.0;
    double transTime = 0.0;

    public enum State{
        OFF(0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        DISABLED(255.0, 20.0, 30.0, Double.POSITIVE_INFINITY, 0.0, false), // solid pink
        ENABLED(0.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false), // solid blue
        EMERGENCY(255.0, 0.0, 0.0, 0.5, 0.5, false), // blinking red
        HOOD_TUCKED(255.0, 20.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false), // solid pink
        TARGET_VISIBLE(0.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false), // solid orange
        TARGET_TRACKING(255.0, 165.0, 0.0, 0.0625, 0.0625, false), // flashing green
        INVISIBLE_TARGET_TRACKING(255.0, 255.0, 0.0, 0.0625, 0.0625, false), // flashing green
        LIMELIGHT_SEES_ONLY(0.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false), // solid green
        CLIMB_MODE(255.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false), // solid purple
        CLIMB_MODE_BUDDY(255.0, 0.0, 255.0, 0.125, 0.125, false), // flashing purple
        EXTENDING(255.0, 165.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false), // solid orange
        EXTENDING_BUDDY(255.0, 165.0, 0.0, 0.125, 0.125, false), // flashing orange
        HUGGING(255.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false), // solid yellow
        HUGGING_BUDDY(255.0, 255.0, 0.0, 0.125, 0.125, false), // flashing yellow
        WRANGLING(0.0, 255.0, 255.0, 0.0625, 0.0625, false), // flashing cyan
        CLIMBING(0.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false), // solid green
        CLIMBING_BUDDY(0.0, 255.0, 0.0, 0.125, 0.125, false), // flashing green
        RAINBOW(0, true),
        BREATHING_PINK(357, 10.0, true);

        double red, green, blue, onTime, offTime, cycleTime, transitionTime;
        float startingHue;
        List<List<Double>> colors = new ArrayList<List<Double>>();
        boolean isCycleColors;
        private State(double r, double g, double b, double onTime, double offTime, boolean isCycleColors){
            red = r / 255.0;
            green = g / 255.0;
            blue = b / 255.0;
            this.onTime = onTime;
            this.offTime = offTime;
        }

        private State(float hue, boolean cycle) {
            this.startingHue = hue;
            this.isCycleColors = cycle;
        }

        private State(float hue, double transTime, boolean cycle) {
            this.startingHue = hue;
            this.transitionTime = transTime;
            this.isCycleColors = cycle;
        }

        private State(List<List<Double>> colors, double cycleTime, boolean isCycleColors, double transitionTime) {
            this.colors = colors;
            this.cycleTime = cycleTime;
            this.isCycleColors = isCycleColors;
            this.transitionTime = transitionTime;
        }
    }

    private State currentState = State.OFF;
    public State getState(){ return currentState; }
    private void setState(State newState){
        if(newState != currentState){
            currentState = newState;
            lastOffTime = 0.0;
            lastOnTime = 0.0;
            lit = false;
        }
    }

    private final Loop loop = new Loop(){

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            
        }

        @Override
        public void onStop(double timestamp) {

        }

    };


    public void setLEDs(double r, double g, double b){
		//A: Green
		//B: Red
        //C: Blue
		canifier.setLEDOutput(r, LEDChannel.LEDChannelB);
		canifier.setLEDOutput(b, LEDChannel.LEDChannelA);
		canifier.setLEDOutput(g, LEDChannel.LEDChannelC);
    }

    public void conformToState(State state){
        setState(state);
    }

    public double stateHue = State.RAINBOW.startingHue;
    public float saturation = 1.0f; // Ensures that the colors are on the outside of the color wheel
    public float value = 0.3f; // Hardcoded brightness
    public double startingTransTime = 0.0;
    public boolean resetBreath = false;

    @Override
    public void writePeriodicOutputs(){
        double timestamp = Timer.getFPGATimestamp();
        if (currentState == State.RAINBOW && currentState.isCycleColors == true) {
            stateHue += 2;
            if (stateHue >= (360 - State.RAINBOW.startingHue)) {
                stateHue = State.RAINBOW.startingHue;
            }

            float rgb[] = new float[3];
            MovingAverage averageR = new MovingAverage(5);
            MovingAverage averageG = new MovingAverage(5);
            MovingAverage averageB = new MovingAverage(5);

            if (saturation > 1) {
                saturation = 1;
            }
            if (saturation < 0) {
                saturation = 0;
            }
            if (value > 1) {
                value = 1;
            }
            if (value < 0) {
                value = 0;
            }
            
            rgb = HSVtoRGB.convert(stateHue, saturation, value);

            rgb[0] = averageR.process(rgb[0]);
            rgb[1] = averageG.process(rgb[1]);
            rgb[2] = averageB.process(rgb[2]);

            setLEDs(rgb[0], rgb[1], rgb[2]);

        } else if (currentState == State.BREATHING_PINK && currentState.isCycleColors == true) {
            if (startingTransTime <= currentState.transitionTime && !resetBreath) {
                startingTransTime += currentState.transitionTime / 50.0;
            } else if (resetBreath) {
                startingTransTime -= currentState.transitionTime / 50.0;
            }
            if (resetBreath && startingTransTime <= 0.0) {
                resetBreath = false;
            } else if (!resetBreath && startingTransTime >= currentState.transitionTime) {
                resetBreath = true;
            }


            float rgb[] = new float[3];
            MovingAverage averageR = new MovingAverage(10);
            MovingAverage averageG = new MovingAverage(10);
            MovingAverage averageB = new MovingAverage(10);

            double valueBasedOnTime = currentState.transitionTime - startingTransTime;
            
            rgb = HSVtoRGB.convert(State.BREATHING_PINK.startingHue, 0.922f, valueBasedOnTime * 0.6);

            rgb[0] = averageR.process(rgb[0]);
            rgb[1] = averageG.process(rgb[1]);
            rgb[2] = averageB.process(rgb[2]);

            setLEDs(rgb[0], rgb[1], rgb[2]);

        } else if(!lit && (timestamp - lastOffTime) >= currentState.offTime && currentState.isCycleColors == false){
            setLEDs(currentState.red, currentState.green, currentState.blue);
            lastOnTime = timestamp;
            lit = true;
        } else if(lit && !Double.isInfinite(currentState.onTime) && currentState.isCycleColors == false){
            if((timestamp - lastOnTime) >= currentState.onTime){
                setLEDs(0.0, 0.0, 0.0);
                lastOffTime = timestamp;
                lit = false;
            }
        } 
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("leds state", getState().name());
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper){
        enabledLooper.register(loop);
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}