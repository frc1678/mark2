package com.team1678.frc2021.planners;

import com.team1678.frc2021.Constants;
import com.team1678.frc2021.planners.IndexerMotionPlanner;
//import com.team1678.frc2021.subsystems.Indexer.ProxyStatus;
import com.team254.util.test.ControlledActuatorLinearSim;
import org.junit.Assert;
import org.junit.Test;

import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class IndexerMotionPlannerTest {

    /*public ProxyStatus setProxyStatus(boolean front_proxy, boolean right_proxy, boolean back_right_proxy, 
            boolean back_left_proxy, boolean left_proxy) { // slots 0, 1, 2, 3, 4
        ProxyStatus proxy_status = new ProxyStatus();
        proxy_status.front_proxy = front_proxy;
        proxy_status.right_proxy = right_proxy;
        proxy_status.left_proxy = left_proxy;
        proxy_status.back_right_proxy = back_right_proxy;
        proxy_status.back_left_proxy = back_left_proxy;

        return proxy_status;
    }*/
    /*
    @Test
    public void testFindNearestOpenSlot() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        double angleGoal = motion_planner.findSnappedAngleToGoal(69);
        Assert.assertEquals(3, angleGoal, Constants.kTestEpsilon);

        ProxyStatus proxy_status = setProxyStatus(false, false, false, false, false);
        int slotGoal = motion_planner.findNearestOpenSlot(72, proxy_status);
        angleGoal = motion_planner.findAngleToIntake(slotGoal, 72);
        Assert.assertEquals(0, angleGoal, Constants.kTestEpsilon);
        Assert.assertEquals(1, slotGoal);
        
        proxy_status = setProxyStatus(true, false, false, false, false);
        slotGoal = motion_planner.findNearestOpenSlot(72, proxy_status);
        angleGoal = motion_planner.findAngleToIntake(slotGoal, 72);
        Assert.assertEquals(72, angleGoal, Constants.kTestEpsilon);
        Assert.assertEquals(2, slotGoal);

        proxy_status = setProxyStatus(true, false, false, false, true);
        slotGoal = motion_planner.findNearestOpenSlot(144, proxy_status);
        angleGoal = motion_planner.findAngleToIntake(slotGoal, 144);
        Assert.assertEquals(72, angleGoal, Constants.kTestEpsilon);
        Assert.assertEquals(3, slotGoal);

        proxy_status = setProxyStatus(true, true, true, false, false);
        slotGoal = motion_planner.findNearestOpenSlot(72, proxy_status);
        angleGoal = motion_planner.findAngleToIntake(slotGoal, 144);
        Assert.assertEquals(-144, angleGoal, Constants.kTestEpsilon);
        Assert.assertEquals(0, slotGoal);
    }*/

    @Test
    public void testSmallOffset() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(-90, 90);
        double angleGoal = motion_planner.findAngleToGoal(slotGoal, -90, 90);
        Assert.assertEquals(3, slotGoal);
        Assert.assertEquals(-36, angleGoal, Constants.kTestEpsilon);
    }
    
    @Test
    public void testBigOffset() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(-135, 135);
        double angleGoal = motion_planner.findAngleToGoal(slotGoal, -135, 135);
        Assert.assertEquals(4, slotGoal);
        Assert.assertEquals(-18, angleGoal, Constants.kTestEpsilon);
    }

    @Test
    public void testMegaOffset() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(-450, 451);
        double angleGoal = motion_planner.findAngleToGoal(slotGoal, -450, 451);
        Assert.assertEquals(3, slotGoal);
        Assert.assertEquals(-35, angleGoal, Constants.kTestEpsilon);
    }

    @Test
    public void testMegaPositiveOffset() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(100, 500);
        double angleGoal = motion_planner.findAngleToGoal(slotGoal, 100, 500);
        Assert.assertEquals(1, slotGoal);
        Assert.assertEquals(-32, angleGoal, Constants.kTestEpsilon);
    }

    @Test
    public void testMegaNegativeOffset() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(-1323, -1678);
        double angleGoal = motion_planner.findAngleToGoal(slotGoal, -1323, -1678);
        Assert.assertEquals(0, slotGoal);
        Assert.assertEquals(5, angleGoal, Constants.kTestEpsilon);
    }

    @Test
    public void testNextSlot() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(-135, 135);
        double angleGoal = motion_planner.findAngleToGoal(slotGoal, -135, 135);
        Assert.assertEquals(4, slotGoal);
        Assert.assertEquals(-18, angleGoal, Constants.kTestEpsilon);

        slotGoal = motion_planner.findNextSlot(-153, 135);
        angleGoal = motion_planner.findAngleToGoal(slotGoal, -153, 135);
        Assert.assertEquals(0, slotGoal);
        Assert.assertEquals(-72, angleGoal, Constants.kTestEpsilon);
    }

    @Test
    public void testPreviousSlot() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(-135, 135);
        double angleGoal = motion_planner.findAngleToGoal(slotGoal, -135, 135);
        Assert.assertEquals(4, slotGoal);
        Assert.assertEquals(-18, angleGoal, Constants.kTestEpsilon);

        slotGoal = motion_planner.findPreviousSlot(-153, 135);
        angleGoal = motion_planner.findAngleToGoal(slotGoal, -153, 135);
        Assert.assertEquals(3, slotGoal);
        Assert.assertEquals(72, angleGoal, Constants.kTestEpsilon);
    }

    @Test
    public void testPrepShoot() {
        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(-90, 90);
        double angleGoal = motion_planner.findNearestDeadSpot(-90, 90);
        Assert.assertEquals(3, slotGoal);
        Assert.assertEquals(0, angleGoal, Constants.kTestEpsilon);
    }

    @Test
    public void testMovingIndexer() {

        ControlledActuatorLinearSim turretSim = new ControlledActuatorLinearSim(-1000, 1000, 10);
        ControlledActuatorLinearSim indexerSim = new ControlledActuatorLinearSim(-1000, 1000, 45);

        turretSim.reset(90);
        indexerSim.reset(0);

        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(0, 90);
        double angleGoal = 0;
        for (double t = 0; t < 10; t += 0.05) {
            indexerSim.setCommandedPosition(angleGoal);
            turretSim.setCommandedPosition(90);
            angleGoal = motion_planner.findAngleGoal(slotGoal, indexerSim.update(0.05), turretSim.update(0.05));
        }
        final double indexer_angle = motion_planner.WrapDegrees(indexerSim.update(0));
        final double turret_angle = motion_planner.WrapDegrees(turretSim.update(0));
        Assert.assertEquals(1, motion_planner.findNearestSlot(indexer_angle, turret_angle));
        Assert.assertTrue(motion_planner.isAtGoal(slotGoal, indexer_angle, turret_angle));
    }

    @Test
    public void testMovingIndexerAndTurret() {

        ControlledActuatorLinearSim turretSim = new ControlledActuatorLinearSim(-1000, 1000, 10);
        ControlledActuatorLinearSim indexerSim = new ControlledActuatorLinearSim(-1000, 1000, 45);

        turretSim.reset(90);
        indexerSim.reset(0);

        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(0, 90);
        double angleGoal = 0;
        for (double t = 0; t < 20; t += 0.05) {
            indexerSim.setCommandedPosition(angleGoal);
            turretSim.setCommandedPosition(190);
            angleGoal = motion_planner.findAngleGoal(slotGoal, indexerSim.update(0.05), turretSim.update(0.05));
        }
        final double indexer_angle = motion_planner.WrapDegrees(indexerSim.update(0));
        final double turret_angle = motion_planner.WrapDegrees(turretSim.update(0));
        Assert.assertEquals(1, motion_planner.findNearestSlot(indexer_angle, turret_angle));
        Assert.assertTrue(motion_planner.isAtGoal(slotGoal, indexer_angle, turret_angle));
    }

    @Test
    public void testMovingIndexerAndTurretOpposite() {

        ControlledActuatorLinearSim turretSim = new ControlledActuatorLinearSim(-1000, 1000, 10);
        ControlledActuatorLinearSim indexerSim = new ControlledActuatorLinearSim(-1000, 1000, 45);
;
        turretSim.reset(90);
        indexerSim.reset(0);

        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(0, 90);
        double angleGoal = 0;
        for (double t = 0; t < 20; t += 0.05) {
            indexerSim.setCommandedPosition(angleGoal);
            turretSim.setCommandedPosition(-10);
            angleGoal = motion_planner.findAngleGoal(slotGoal, indexerSim.update(0.05), turretSim.update(0.05));
        }
        final double indexer_angle = motion_planner.WrapDegrees(indexerSim.update(0));
        final double turret_angle = motion_planner.WrapDegrees(turretSim.update(0));
        System.out.println(indexer_angle + " " + turret_angle + " " + angleGoal);
        Assert.assertEquals(1, motion_planner.findNearestSlot(indexer_angle, turret_angle));
        Assert.assertTrue(motion_planner.isAtGoal(slotGoal, indexer_angle, turret_angle));
    }

    @Test
    public void testMovingIndexerAndTurretReflexively() {

        ControlledActuatorLinearSim turretSim = new ControlledActuatorLinearSim(-1000, 1000, 10);
        ControlledActuatorLinearSim indexerSim = new ControlledActuatorLinearSim(-1000, 1000, 45);

        turretSim.reset(0);
        indexerSim.reset(0);

        IndexerMotionPlanner motion_planner = new IndexerMotionPlanner();
        int slotGoal = motion_planner.findNearestSlot(0, 0);
        double angleGoal = 0;
        for (double t = 0; t < 69; t += 0.05) {
            indexerSim.setCommandedPosition(angleGoal);
            turretSim.setCommandedPosition(-360);
            angleGoal = motion_planner.findAngleGoal(slotGoal, indexerSim.update(0.05), turretSim.update(0.05));
        }
        final double indexer_angle = motion_planner.WrapDegrees(indexerSim.update(20));
        final double turret_angle = motion_planner.WrapDegrees(turretSim.update(0));
        Assert.assertEquals(0, motion_planner.findNearestSlot(indexer_angle, turret_angle));
        Assert.assertTrue(motion_planner.isAtGoal(slotGoal, indexer_angle, turret_angle));
    }
}