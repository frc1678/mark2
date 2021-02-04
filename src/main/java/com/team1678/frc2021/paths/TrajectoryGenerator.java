package com.team1678.frc2021.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team1323.lib.trajectory.TrajectoryUtil;
import com.team1323.lib.trajectory.Trajectory;
import com.team1323.lib.trajectory.timing.TimedState;
import com.team1323.lib.trajectory.timing.TimingConstraint;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.planners.DriveMotionPlanner;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 120.0;
    private static final double kMaxDecel = 72.0;
    private static final double kMaxVoltage = 9.0;
    
    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;
    
    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }
    
    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }
    
    public void generateTrajectories() {
        if(mTrajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }
    
    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }
    
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
    boolean reversed,
    final List<Pose2d> waypoints,
    final List<TimingConstraint<Pose2dWithCurvature>> constraints,
    double max_vel,  // inches/s
    double max_accel,  // inches/s^2
    double max_decel,
    double max_voltage,
    double default_vel,
    int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
        default_vel, slowdown_chunks);
    }
    
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
    boolean reversed,
    final List<Pose2d> waypoints,
    final List<TimingConstraint<Pose2dWithCurvature>> constraints,
    double start_vel,  // inches/s
    double end_vel,  // inches/s
    double max_vel,  // inches/s
    double max_accel,  // inches/s^2
    double max_decel,
    double max_voltage,
    double default_vel,
    int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
        default_vel, slowdown_chunks);
    }
    
    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
    static final Pose2d autoStartingPose = new Pose2d(Constants.kRobotStartingPose.getTranslation().translateBy(new Translation2d(0.0, 0.0)), Rotation2d.fromDegrees(0.0));
    
    static final Pose2d firstTrenchIntakePose = Constants.kFirstTrenchBall.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d thirdTrenchIntakePose = Constants.kLastTrenchBall.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d trenchPairIntakePose = Constants.kTrenchBallSet.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d trenchOppositePairIntakePose = Constants.kOppositeTrenchBalls.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d trenchCornerPose = new Pose2d(new Translation2d(208.0, -108.5), Rotation2d.fromDegrees(-25.0));
    
    static final Pose2d generatorIntake1Pose = Constants.kGeneratorBall1.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    
    
    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }
            
            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }
            
            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }
        
        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> straightPath;
        
        //Preliminary Auto Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToGenerator1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> generator1ToStart;
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToTrenchRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchRunToStart;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchRunToCorner;

        public final Trajectory<TimedState<Pose2dWithCurvature>> generator1ToTrenchRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> generator1ToTrenchCorner;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchCornerToTrenchRun;
        
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToOppoTrench;
        public final Trajectory<TimedState<Pose2dWithCurvature>> oppoTrenchToStart;
        
        private TrajectorySet() {
            //Test Paths
            straightPath = getStraightPath();
            
            //Preliminary Auto Paths
            startToGenerator1 = getStartToGenerator1();
            generator1ToStart = getGenerator1ToStart();
            startToTrenchRun = getStartToTrenchRun();
            trenchRunToStart = getTrenchRunToStart();
            trenchRunToCorner = getTrenchRunToCorner();

            generator1ToTrenchRun = getGenerator1ToTrenchRun();
            generator1ToTrenchCorner = getGenerator1ToTrenchCorner();
            trenchCornerToTrenchRun = getTrenchCornerToTrenchRun();
            
            startToOppoTrench = getStartToOppoTrench();
            oppoTrenchToStart = getOppoTrenchToStart();
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.kRobotStartingPose);
            waypoints.add(Constants.kRobotStartingPose.transformBy(Pose2d.fromTranslation(new Translation2d(72.0, 0.0))));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToGenerator1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-30.0)));
            waypoints.add(generatorIntake1Pose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator1ToStart() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(generatorIntake1Pose);
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-30.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToTrenchRun() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-50.0)));
            waypoints.add(firstTrenchIntakePose);
            waypoints.add(trenchPairIntakePose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchRunToStart() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(trenchPairIntakePose);
            waypoints.add(firstTrenchIntakePose);
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-50.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchRunToCorner() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(trenchPairIntakePose);
            waypoints.add(thirdTrenchIntakePose);
            waypoints.add(trenchCornerPose);
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator1ToTrenchRun() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(generatorIntake1Pose.getTranslation(), generatorIntake1Pose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0))));
            waypoints.add(firstTrenchIntakePose);
            waypoints.add(trenchPairIntakePose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator1ToTrenchCorner() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(generatorIntake1Pose.getTranslation(), generatorIntake1Pose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0))));
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation(), Rotation2d.fromDegrees(-90.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchCornerToTrenchRun() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation(), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(firstTrenchIntakePose);
            waypoints.add(trenchPairIntakePose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToOppoTrench() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(90.0)));
            waypoints.add(trenchOppositePairIntakePose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getOppoTrenchToStart() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(90.0)));
            waypoints.add(trenchOppositePairIntakePose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        
    }
}
