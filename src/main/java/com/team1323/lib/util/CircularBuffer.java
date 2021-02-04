/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.lib.util;

import java.util.LinkedList;

/**
 * Add your docs here.
 */
public class CircularBuffer {
    int mWindowSize;
    LinkedList<Double> mSamples;
    double mSum;
    
    public CircularBuffer(int window_size) {
        mWindowSize = window_size;
        mSamples = new LinkedList<Double>();
        mSum = 0.0;
    }
    
    public void clear() {
        mSamples.clear();
        mSum = 0.0;
    }
    
    public double getAverage() {
        if (mSamples.isEmpty())
        return 0.0;
        return mSum / mSamples.size();
    }
    
    public void recomputeAverage() {
        // Reset any accumulation drift.
        mSum = 0.0;
        if (mSamples.isEmpty())
        return;
        for (Double val : mSamples) {
            mSum += val;
        }
        mSum /= mWindowSize;
    }
    
    public void addValue(double val) {
        mSamples.addLast(val);
        mSum += val;
        if (mSamples.size() > mWindowSize) {
            mSum -= mSamples.removeFirst();
        }
    }
    
    public int getNumValues() {
        return mSamples.size();
    }
    
    public boolean isFull() {
        return mWindowSize == mSamples.size();
    }
}
