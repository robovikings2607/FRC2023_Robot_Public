package frc.robot.commands.AutonCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.commands.ArmCommands.SequentialArmUp;
import frc.robot.commands.ArmCommands.SetArmPreset;
import frc.robot.commands.AutonCommands.AutonUtilities.ResetOdometry;
import frc.robot.commands.IntakeCommands.ActuateHybridIntake;
import frc.robot.commands.IntakeCommands.Drop;
import frc.robot.commands.IntakeCommands.HybridIntakeCommand;
import frc.robot.commands.IntakeCommands.HybridOuttake;
import frc.robot.commands.IntakeCommands.SetIntakeState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.RVAprilTagField;

public class ThreePieceStartCone extends SequentialCommandGroup{
    private DriveSubsystem driveSubsystem;
    private int allianceSign;
    Rotation2d startingRot;

    static final TrapezoidProfile.Constraints SPEEDY_CONSTRAINTS = 
        new TrapezoidProfile.Constraints(3.5, 2);
    static final TrapezoidProfile.Constraints SLOWER_DECEL_CONSTRAINTS = 
        new TrapezoidProfile.Constraints(1.8, 1.8);

    public ThreePieceStartCone(RobotContainer robot){
        driveSubsystem = robot.swerveDrive;
        addRequirements(driveSubsystem);

        allianceSign = (DriverStation.getAlliance() == Alliance.Blue) ? -1 : 1;

        startingRot = new Rotation2d((Math.PI/2) - (allianceSign*(Math.PI/2)));

        Translation2d startPose = 
            new Translation2d(6.43*allianceSign, 0.975);

        Translation2d shootCubePose =
            new Translation2d(
                (RVAprilTagField.CHARGE_STATION_EDGE_X + Units.inchesToMeters(70.0)) * allianceSign,
                startPose.getY() - Units.inchesToMeters(6.0)
            );

        Translation2d intermediaryPose =
         new Translation2d( RVAprilTagField.CHARGE_STATION_EDGE_X * allianceSign,
                            RVAprilTagField.TOP_PIECE_Y - Units.inchesToMeters(6.0));

        addCommands(
            new ResetOdometry(
                driveSubsystem, 
                new Pose2d(startPose, startingRot)
            ),
            new SetIntakeState(robot, true),
            new SequentialArmUp(robot, ArmPresets.SCORING_HIGH).yield(),
            new Drop(robot),
            new ActuateHybridIntake(robot, !IntakeConstants.HYBRID_INTAKE_RETRACTED),

            new ParallelRaceGroup(  //picking up first piece
                new HybridIntakeCommand(robot).withTimeout(5.0),
                new ParallelCommandGroup(
                    new WaitCommand(0.5).andThen(new SetArmPreset(robot, ArmPresets.STOWING)),
                    // new SequentialCommandGroup(
                        // new GoToPoint(robot,
                        //     intermediaryPose, 
                        //     startingRot, 
                        //     0.0, 3.0),//.withLinearConstraints(SPEEDY_CONSTRAINTS), //MADE SPEEDY
                        new GoToPoint(
                            robot, 
                            new Translation2d(
                                RVAprilTagField.PIECE_X * allianceSign,
                                RVAprilTagField.TOP_PIECE_Y - Units.inchesToMeters(16.0)
                            ),
                            startingRot
                            /*3.0, 0.0*/)
                        .withLinearConstraints(SPEEDY_CONSTRAINTS)
                    // )
                )
            ),

            new ActuateHybridIntake(robot, IntakeConstants.HYBRID_INTAKE_RETRACTED),

            //scoring second cube
            new GoToPoint(robot,
                intermediaryPose,
                startingRot.plus(new Rotation2d(Math.PI)),
                0.0, 2.0),//.withLinearConstraints(SPEEDY_CONSTRAINTS),//MADE SPEEDY //vf was 3.0
            new GoToPoint(robot,
                shootCubePose,
                startingRot.plus(new Rotation2d(Math.PI)),
                2.0, 0.0),//.withLinearConstraints(SPEEDY_CONSTRAINTS), //MADE SPEEDY,
            // new GoToPoint(robot, 
            //     scoringMidPose, 
            //     startingRot.plus(new Rotation2d(Math.PI)), 3.0, 0.0
            // ).withLinearConstraints(SPEEDY_CONSTRAINTS),
            new HybridOuttake(robot, true).withTimeout(0.5)
            // BELOW MODIFIED
        //     new GoToPoint(
        //         robot, 
        //         intermediaryPose.plus(new Translation2d(Units.inchesToMeters(6) * allianceSign, 0)),
        //         startingRot.plus(new Rotation2d(Math.PI/4*allianceSign)),
        //         0.0,
        //         2.0
        //     ).setAngleTimeOffset(0.0).withLinearConstraints(SPEEDY_CONSTRAINTS),

        //     new ActuateHybridIntake(robot, !IntakeConstants.HYBRID_INTAKE_RETRACTED),

        //     new ParallelRaceGroup(
        //         new HybridIntakeCommand(robot).withTimeout(5.0),
        //         new GoToPoint(
        //             robot, 
        //             new Translation2d(
        //                 (RVAprilTagField.PIECE_X - Units.inchesToMeters(6)) * allianceSign,
        //                 RVAprilTagField.TOP_MID_PIECE_Y), 
        //             startingRot.plus(new Rotation2d(Math.PI/12*allianceSign)),
        //             2.0,
        //             0.0
        //         )//.withLinearConstraints(SPEEDY_CONSTRAINTS)//MADE SPEEDY
        //     ),

        //     new ActuateHybridIntake(robot, IntakeConstants.HYBRID_INTAKE_RETRACTED),

        //     //scoring third piece
        //     new GoToPoint(
        //         robot, 
        //         intermediaryPose,
        //         // new Translation2d(
        //         //     RVAprilTagField.CHARGE_STATION_EDGE_X * allianceSign,
        //         //     scoringTopPose.getY()),
        //         startingRot.plus(new Rotation2d(Math.PI/4*allianceSign)),
        //         0.0,
        //         2.0 //vf was 3.0
        //     ),//.withLinearConstraints(SPEEDY_CONSTRAINTS), //MADE SPEEDY
        //     // new GoToPoint(
        //     //     robot, 
        //     //     startPose.minus(new Translation2d(Units.inchesToMeters(20)*allianceSign, Units.inchesToMeters(-8))),
        //     //     startingRot.plus(new Rotation2d(Math.PI)), 
        //     //     3.0,
        //     //     1.5
        //     // ),
        //     // new GoToPoint(
        //     //     robot, 
        //     //     scoringTopPose, 
        //     //     startingRot.plus(new Rotation2d(Math.PI)),
        //     //     1.5,
        //     //     0.0
        //     // ),
        //     new GoToPoint(robot,
        //         shootCubePose,
        //         startingRot.plus(new Rotation2d(Math.PI)),
        //         2.0, 0.0),//.withLinearConstraints(SPEEDY_CONSTRAINTS), //MADE SPEEDY

        //     new HybridOuttake(robot, true)
        );
    }
}