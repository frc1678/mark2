// package com.team1678.frc2021.auto;

// import java.util.List;

// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// import com.team1678.frc2021.Constants;
// import com.team1678.frc2021.auto.actions.SwerveTrajectoryAction;
// import com.team1678.frc2021.subsystems.Swerve;
// import com.team1678.frc2021.commands.AutoAimCommand;
// import com.team1678.frc2021.commands.IntakeCommand;
// import com.team1678.frc2021.commands.ReadyGyro;
// import com.team1678.frc2021.commands.ShootCommand;
// import com.team1678.frc2021.commands.SpinUpCommand;
// import com.team1678.frc2021.commands.SwervePointTurnCommand;
// import com.team1678.frc2021.commands.TuckCommand;
// import com.team1678.frc2021.commands.WaitToAutoAimCommand;
// import com.team1678.frc2021.commands.WaitToIntakeCommand;
// import com.team1678.frc2021.commands.WaitToSpinUpCommand;
// import com.team1678.frc2021.subsystems.Intake;
// import com.team1678.frc2021.subsystems.Superstructure;

// public class LeftSixFarMode extends AutoModeBase {

//     // Swerve instance 
//     private final Swerve s_Swerve = new Swerve();

//     // trajectory actions
//     SwerveTrajectoryAction leftSixFirstShotTrajectory;
//     SwerveTrajectoryAction leftSixIntakeTrajectory;
//     SwerveTrajectoryAction leftSixSecondShotTrajectory;

//     public LeftSixFarMode() {

//         final Intake mIntake = Intake.getInstance();
//         final Superstructure mSuperstructure = Superstructure.getInstance();

//         var thetaController =
//             new ProfiledPIDController(
//                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         /** TRAJECTORY DEFINITIONS **/
    
//         Trajectory leftSixFirstShot =
//             TrajectoryGenerator.generateTrajectory(
//                 new Pose2d(2.9, 7.5, Rotation2d.fromDegrees(0.0)),
//                 List.of(),
//                 new Pose2d(4.8, 7.5 , Rotation2d.fromDegrees(0.0)),
//                 Constants.AutoConstants.defaultConfig);

//         Trajectory leftSixIntake =          
//             TrajectoryGenerator.generateTrajectory(
//                 new Pose2d(5.3, 7.5 , Rotation2d.fromDegrees(0.0)),
//                 List.of(),
//                 new Pose2d(8.2, 7.5, Rotation2d.fromDegrees(0.0)),
//                 Constants.AutoConstants.defaultConfig);

//         Trajectory leftSixSecondShot =          
//             TrajectoryGenerator.generateTrajectory(
//                 new Pose2d(8.2, 7.5 , Rotation2d.fromDegrees(180.0)),
//                 List.of(),
//                 new Pose2d(5.3, 7.5, Rotation2d.fromDegrees(180.0)),
//                 Constants.AutoConstants.defaultConfig);
    
//         /** SWERVE TRAJECTORY ACTION DEFINITIONS **/

//         leftSixFirstShotTrajectory =
//             new SwerveTrajectoryAction(
//                 leftSixFirstShot,
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                 thetaController,
//                 () -> Rotation2d.fromDegrees(0),
//                 s_Swerve::setModuleStates);

//         leftSixIntakeTrajectory =
//             new SwerveTrajectoryAction(
//                 leftSixIntake,
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                 thetaController,
//                 () -> Rotation2d.fromDegrees(0),
//                 s_Swerve::setModuleStates);

//         leftSixSecondShotTrajectory =
//             new SwerveTrajectoryAction(
//                 leftSixSecondShot,
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                 thetaController,
//                 () -> Rotation2d.fromDegrees(0),
//                 s_Swerve::setModuleStates);

//         SwervePointTurnCommand endAdjustCommand =
//             new SwervePointTurnCommand(
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                 thetaController,
//                 () -> Rotation2d.fromDegrees(180),
//                 s_Swerve::setModuleStates);

//         IntakeCommand intake = 
//             new IntakeCommand(mIntake, mSuperstructure);

//         SpinUpCommand spinUp = 
//             new SpinUpCommand(mSuperstructure);
            
//         ShootCommand firstShoot =
//             new ShootCommand(mSuperstructure);

//         ShootCommand secondShoot =
//             new ShootCommand(mSuperstructure);

//         AutoAimCommand firstAim =
//             new AutoAimCommand(mSuperstructure, 200);

//         AutoAimCommand secondAim =
//             new AutoAimCommand(mSuperstructure, 200);

//         WaitToSpinUpCommand waitToSpinUp = 
//             new WaitToSpinUpCommand(mSuperstructure, 1.5);

//         WaitToAutoAimCommand waitToAutoAim = 
//             new WaitToAutoAimCommand(mSuperstructure, 200, 1.5);

//         WaitToIntakeCommand waitToFirstIntake = 
//             new WaitToIntakeCommand(mIntake, mSuperstructure, 1.5);

//         ReadyGyro readyGyro = 
//             new ReadyGyro(s_Swerve);

//         addCommands(
//             new InstantCommand(() -> s_Swerve.resetOdometry(leftSixFirstShot.getInitialPose())),
//             new SequentialCommandGroup(
//                 leftSixFirstShotCommand.deadlineWith(
//                     waitToAutoAim,
//                     waitToSpinUp,
//                     waitToFirstIntake
//                 ),
//                 firstShoot,
//                 leftSixIntakeCommand.deadlineWith(intake),
//                 leftSixSecondShotCommand.deadlineWith(new SequentialCommandGroup(
//                     new WaitCommand(1.0),
//                     (secondAim)
//                 )),
//                 secondShoot
//             ),
//             readyGyro
//         ); 


//     }
    
// }
