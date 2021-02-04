package com.team1678.lib.util;

public class HallCalibration {

    /*
     * The max and min raw sensor values of when the hall sensor reads true, as seen
     * so far. When calibration is complete it is approximately the top and bottom
     * of the magnet's range.
     */

    double mMaxHallTrue = 0;
    double mMinHallTrue = 0;

    // The max and min raw sensor values that have been read
    double mMaxOverall = 0;
    double mMinOverall = 0;

    // Whether it is the first time running Update()
    boolean mFirstTime = true;

    // Whether the hall sensor has ever been triggered
    boolean mMagnetFound = false;
    boolean mCalibrated = false;

    // The offset is the value *added* to the raw sensor values
    double offset = 0;

    // The value that should be returned at the center of the magnet
    double mMagnetPosition;

    public HallCalibration(double reset_position) {
        mMagnetPosition = reset_position;

    }

    public double update(double main_sensor_value, boolean hall_value) {
        if (hall_value) {
            /*
             * Update the max and min values for when the hall sensor is triggered. Set them
             * to the current value if it is the first time seeing the magnet.
             */

            if (main_sensor_value > mMaxHallTrue || !mMagnetFound) {
                mMaxHallTrue = main_sensor_value;
            }
            if (main_sensor_value < mMinHallTrue || !mMagnetFound) {
                mMinHallTrue = main_sensor_value;
            }
            mMagnetFound = true;
        }

        /*
         * Update the max and min overall values. Set the to the current value if it is
         * the first time running Update()
         */

        if (main_sensor_value > mMaxOverall || mFirstTime) {
            mMaxOverall = main_sensor_value;
        }
        if (main_sensor_value < mMinOverall || mFirstTime) {
            mMinOverall = main_sensor_value;
        }

        /*
         * Return the best estimate known. If the magnet is not found or the edges of
         * the magnet's range have not been reached, there is no best estimate. In the
         * event that mCalibrated is true, do not set it to false even if the //
         * condition is not currently met, as that could reset any portions of code that
         * assume calibration is complete.
         */

        if ((mMagnetFound && mMaxOverall > mMaxHallTrue && mMinOverall < mMinHallTrue) || mCalibrated) {
            /*
             * The center of the magnet's range is mMagnetPosition, so the offset if
             * mMagnetPosition - the raw value of the center of the magnet
             */
            offset = mMagnetPosition - (mMaxHallTrue + mMinHallTrue) / 2;
            mCalibrated = true;
        }

        mFirstTime = false;
        return main_sensor_value + offset;
    }

    public boolean isCalibrated() {
        return mCalibrated;
    }

    public double getOffset() {
        return offset;
    }
}
