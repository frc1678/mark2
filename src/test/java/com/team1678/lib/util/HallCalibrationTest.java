package com.team1678.lib.util;

import org.junit.Assert;
import org.junit.Test;

import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)

public class HallCalibrationTest {

    public HallCalibration calibration_ = new HallCalibration(0);

    public void updateTest(double position) {
        // The magnet's range is 100 to 200 exclusive
        if (position > 100 && position < 200) {
            calibration_.update(position, true);
        } else {
            calibration_.update(position, false);
        }
    }

    // update the calibration allowing for sensors being at different positions

    public void updateTest(double main_sensor_position, double hall_sensor_position) {
        if (hall_sensor_position > 100 && hall_sensor_position < 200) {
            calibration_.update(main_sensor_position, true);
        } else {
            calibration_.update(main_sensor_position, false);
        }
    }

    public boolean isCalibrated() {
        return calibration_.isCalibrated();
    }

    // Get the offsetted value from a raw sensor value

    public double ValueAt(double position) {
        return calibration_.update(position, false);
    }

    @Test

    public void SanityCheck() {
        calibration_ = new HallCalibration(0);
        Assert.assertFalse(isCalibrated());
    }

    @Test

    public void CalibratesGoingUp() {
        calibration_ = new HallCalibration(0);
        for (int i = 0; i < 200; i++) {
            updateTest(i);
            Assert.assertFalse(isCalibrated());
        }
        updateTest(200);
        Assert.assertTrue(isCalibrated());
    }

    /*
     * Test that it calibrates going down, using all decreasing values
     */
    @Test
    public void CalibratesGoingDown() {
        calibration_ = new HallCalibration(0);
        for (int i = 300; i > 100; i--) {
            updateTest(i);
            Assert.assertFalse(isCalibrated());
        }
        updateTest(100);
        Assert.assertTrue(isCalibrated());
    }

    /*
     * Starting from outside the magnet's range, test that going into the magnet's
     * range and then reversing back out does not calibrate it, and that going
     * through the magnet's range after that successfully calibrates it.
     */
    @Test
    public void ReverseFromOutside() {
        calibration_ = new HallCalibration(0);
        for (int i = 0; i < 150; i++) {
            updateTest(i);
            Assert.assertFalse(isCalibrated());
        }
        for (int i = 150; i > 0; i--) {
            updateTest(i);
            Assert.assertFalse(isCalibrated());
        }
        for (int i = 0; i < 200; i++) {
            updateTest(i);
            Assert.assertFalse(isCalibrated());
        }
        updateTest(200);
        Assert.assertTrue(isCalibrated());
    }

    /*
     * Test that starting in the magnet's range and moving out does not calibrate
     * it, and that going through the magnet's range after that successfully
     * calibrates it.
     */
    @Test
    public void ReverseFromInside() {
        calibration_ = new HallCalibration(0);
        for (int i = 150; i > 0; i--) {
            updateTest(i);
            Assert.assertFalse(isCalibrated());
        }
        for (int i = 0; i < 200; i++) {
            updateTest(i);
            Assert.assertFalse(isCalibrated());
        }
        updateTest(200);
        Assert.assertTrue(isCalibrated());
    }

    /*
     * Test that the calibration routine finds the center of the magnet's range
     * under normal conditions.
     */
    @Test
    public void FindsMagnetCenter() {
        calibration_ = new HallCalibration(0);
        for (int i = 0; i < 200; i++) {
            updateTest(i);
        }
        updateTest(200);
        Assert.assertTrue(isCalibrated());
        Assert.assertEquals(ValueAt(150), 0, 1);
    }

    /*
     * Test that a nonzero value can be used as the center of the magnet
     */
    @Test
    public void UsesMagnetPosition() {
        calibration_ = new HallCalibration(1000);
        for (int i = 0; i < 200; i++) {
            updateTest(i);
        }
        updateTest(200);
        Assert.assertTrue(isCalibrated());
        Assert.assertEquals(ValueAt(150), 1000, 1);
    }

    /*
     * Test that lack of rigidity does not make the calibration get too far off.
     * This is an issue because it can cause the sensors to be turning different
     * directions when the mechanism reverses quickily. Various tests are required
     * to cover this.
     *
     * SensorInaccuracies1 tests that when the hall sensor passes completely by the
     * magnet and the main sensor doesn't, calibration is not affected
     */
    @Test
    public void SensorInaccuracies1() {
        calibration_ = new HallCalibration(0);
        for (int i = 0; i < 195; i++) {
            updateTest(i, i);
        }
        // The main sensor has reversed direction, but the hall hasn't
        for (int i = 0; i < 10; i++) {
            updateTest(195 - i, 195 + i);
        }
        Assert.assertFalse(isCalibrated());
        // The hall has reversed direction and is in sync with the main sensor
        for (int i = 185; i > 180; i--) {
            updateTest(i, i);
        }
        for (int i = 180; i < 200; i++) {
            updateTest(i, i);
        }
        updateTest(200, 200);
        Assert.assertTrue(isCalibrated());
        Assert.assertEquals(ValueAt(150), 0, 1);
    }

    /*
     * SensorInaccuracies2 tests that when the hall sensor temporarily enters into
     * range of the magnet, calibration is not significantly affected
     */
    @Test
    public void SensorInaccuracies2() {
        calibration_ = new HallCalibration(0);
        for (int i = 0; i < 95; i++) {
            updateTest(i, i);
        }
        // The main sensor has reversed direction, but the hall hasn't
        for (int i = 0; i < 10; i++) {
            updateTest(95 - i, 95 + i);
        }
        // The hall has reversed direction and is in sync with the main sensor
        for (int i = 85; i > 0; i--) {
            updateTest(i, i);
        }
        // Now calibrate normally
        for (int i = 0; i < 200; i++) {
            updateTest(i, i);
        }
        updateTest(200, 200);
        Assert.assertTrue(isCalibrated());
        Assert.assertEquals(ValueAt(150), 0, 10);
    }

    /*
     * Test that even if the condition for being calibrated becomes false, it does
     * not become uncalibrated.
     */
    @Test
    public void DoesntUncalibrate() {
        HallCalibration calibration = new HallCalibration(0);
        // Calibrate normally
        // Don't use the fixture because this doesn't really follow any model
        for (int i = 0; i < 100; i++) {
            calibration.update(i, false);
        }
        for (int i = 100; i < 200; i++) {
            calibration.update(i, true);
        }
        for (int i = 200; i < 300; i++) {
            calibration.update(i, false);
            // It currently meets the condition for being calibrated
            Assert.assertTrue(calibration.isCalibrated());
        }
        for (int i = 300; i < 400; i++) {
            calibration.update(i, true);
            // It currently does not meet the condition for being calibrated, but it
            // was previously calibrated
            Assert.assertTrue(calibration.isCalibrated());
        }
        calibration.update(400, false);
        // It now meets the condition for being calibrated
        Assert.assertTrue(calibration.isCalibrated());
    }
}