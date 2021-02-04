package com.team1678.frc2021.paths;

import com.team1678.frc2021.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Util;
import org.junit.Assert;
import org.junit.Test;

import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

public class TrajectoryGeneratorTest {
    public static final double kTestEpsilon = 1e-5;

    public void verifyTrajectory(final Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
                                           boolean shouldBeReversed) {
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> iterator = new TrajectoryIterator<>(new TimedView<>
                (trajectory));

        final double dt = 0.05;
        TimedState<Pose2dWithCurvature> prev = null;
        while (!iterator.isDone()) {
            TimedState<Pose2dWithCurvature> state = iterator.getState();

            Assert.assertTrue((shouldBeReversed ? -1.0 : 1.0) * state.velocity() >= -kTestEpsilon);

            if (prev != null) {
                // Check there are no angle discontinuities.
                final double kMaxReasonableChangeInAngle = 0.3;  // rad
                Twist2d change = Pose2d.log(prev.state().getPose().inverse().transformBy(state.state()
                        .getPose()));
                Assert.assertTrue(Math.abs(change.dtheta) < kMaxReasonableChangeInAngle);
                if (!Util.epsilonEquals(change.dtheta, 0.0)) {
                    // Could be a curvature sign change between prev and now, so just check that either matches our
                    // expected sign.
                    final boolean curvature_positive = state.state().getCurvature() > kTestEpsilon ||
                            prev.state().getCurvature() > kTestEpsilon;
                    final boolean curvature_negative = state.state().getCurvature() < -kTestEpsilon ||
                            prev.state().getCurvature() < -kTestEpsilon;
                    final double actual_curvature = change.dtheta / change.dx;
                    if (actual_curvature < -kTestEpsilon) {
                        Assert.assertTrue(curvature_negative);
                    } else if (actual_curvature > kTestEpsilon) {
                        Assert.assertTrue(curvature_positive);
                    }
                }
            }

            iterator.advance(dt);
            prev = state;
        }
        Assert.assertTrue(iterator.isDone());
        System.out.println("t: " + iterator.getSample().state().t());
    }

    @Test
    public void test() {
        TrajectoryGenerator.getInstance().generateTrajectories();
        //verifyTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().testPathReversed, true);
    }
}