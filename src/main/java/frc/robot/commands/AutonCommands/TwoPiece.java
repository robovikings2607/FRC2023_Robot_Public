package frc.robot.commands.AutonCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.commands.ArmCommands.SetArmPreset;
import frc.robot.commands.AutonCommands.AutonUtilities.ResetOdometry;
import frc.robot.commands.IntakeCommands.Drop;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.SetIntakeState;
import frc.robot.subsystems.DriveSubsystem;


public class TwoPiece extends SequentialCommandGroup {

  private DriveSubsystem driveSubsystem;
  private int allianceSign;
  Rotation2d startingRot;


  static final TrapezoidProfile.Constraints SPEEDY_CONSTRAINTS = new TrapezoidProfile.Constraints(1.8, 2);
  static final TrapezoidProfile.Constraints SLOWER_DECEL_CONSTRAINTS = new TrapezoidProfile.Constraints(1.8, 1.8);
  
  public TwoPiece(RobotContainer robot) {
    driveSubsystem = robot.swerveDrive;
    addRequirements(driveSubsystem);

    allianceSign = (DriverStation.getAlliance() == Alliance.Blue) ? -1 : 1;

    startingRot = new Rotation2d((Math.PI/2) - (allianceSign*(Math.PI/2)));

    Translation2d startPose = new Translation2d((6.43-Units.inchesToMeters(20.0))*allianceSign, 0.31);



    addCommands(
      new ResetOdometry(
        driveSubsystem, 
        new Pose2d(startPose, startingRot)),
       new SetIntakeState(robot, false),
       new SetArmPreset(robot, ArmPresets.SCORING_HIGH).yield(),
       new Drop(robot),
       new SetArmPreset(robot, ArmPresets.STOWING)
    );

    double transitionSpeed = 1;
    // 10.5 blue-side maybe
    GoToPoint rotateOut = 
        new GoToPoint(
          robot,
          startPose.minus(new Translation2d(Units.inchesToMeters(14)*allianceSign, -Units.inchesToMeters(10))),
          startingRot, 0, transitionSpeed).withLinearConstraints(SPEEDY_CONSTRAINTS);
   
    GoToPoint midfield = 
        new GoToPoint(
          robot,
          startPose.minus(new Translation2d(Units.inchesToMeters(195)*allianceSign, -Units.inchesToMeters(10))),
          startingRot.plus(new Rotation2d(Math.PI)), transitionSpeed, 0).setAngleTimeOffset(2).withLinearConstraints(SLOWER_DECEL_CONSTRAINTS);

    addCommands(
       // Go to midfield
       new ParallelCommandGroup(new Command[] {
        new SequentialCommandGroup(new Command[] {
          new WaitCommand(1.5),
          new SetArmPreset(robot, ArmPresets.INTAKING).withTimeout(2),
          new IntakeCommand(robot)
        }),
        new SequentialCommandGroup(new Command[] {
          rotateOut,
          midfield
        })
       }),
       new ParallelCommandGroup(new Command[] {
        new SetArmPreset(robot, ArmPresets.STOWING).withTimeout(2),
        new GoToPoint(robot, startPose, startingRot).withLinearConstraints(SPEEDY_CONSTRAINTS).setAngleTimeOffset(2),
       }),
       new SetArmPreset(robot, ArmPresets.SCORING_MID).withTimeout(1),
       new Drop(robot),
       new SetArmPreset(robot, ArmPresets.STOWING)
    );
  }
}
