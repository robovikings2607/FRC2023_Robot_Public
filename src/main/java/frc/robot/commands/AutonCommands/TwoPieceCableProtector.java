// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
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
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.SetIntakeState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.RVAprilTagField;


public class TwoPieceCableProtector extends SequentialCommandGroup {

  private DriveSubsystem driveSubsystem;
  private int allianceSign;
  Rotation2d startingRot;
  // private static DriverStation DS;

  public TwoPieceCableProtector(RobotContainer robot) {
    driveSubsystem = robot.swerveDrive;
    addRequirements(driveSubsystem);


    allianceSign = (DriverStation.getAlliance() == Alliance.Blue) ? -1 : 1;

    startingRot = new Rotation2d((Math.PI/2) - (allianceSign*(Math.PI/2)));

    Translation2d piecePose = new Translation2d(
      RVAprilTagField.PIECE_X * allianceSign,
      RVAprilTagField.LOW_PIECE_Y
    );

    Translation2d startingPose = new Translation2d(6.43*allianceSign, 0.975 - (Units.inchesToMeters(22)*8));
    final TrapezoidProfile.Constraints SPEEDY_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxSpeedMetersPerSecond, 1.9);
    final TrapezoidProfile.Constraints MAX_ANGULAR_CONSTRAINTS = new TrapezoidProfile.Constraints(360, 360*2);

    addCommands(
      new ResetOdometry(driveSubsystem, new Pose2d(startingPose, startingRot)),
      new SetIntakeState(robot, true),
      new SequentialArmUp(robot, ArmPresets.SCORING_HIGH).yield(),
      new Drop(robot),
      new ActuateHybridIntake(robot, !IntakeConstants.HYBRID_INTAKE_RETRACTED),
      new ParallelRaceGroup(
        new HybridIntakeCommand(robot).withTimeout(5),
        new SequentialCommandGroup(
          new WaitCommand(0.5).andThen(new SetArmPreset(robot, ArmPresets.STOWING)),
          new GoToPoint(
            robot,
            piecePose,
            startingRot
          )
        )
      ),
      new GoToPoint(robot, piecePose, startingRot.plus(Rotation2d.fromDegrees(180))),
      new GoToPoint(
        robot, 
        piecePose.plus(new Translation2d(allianceSign, 0)),
        startingRot.plus(Rotation2d.fromDegrees(180))
      ),
      new HybridOuttake(robot, true)
    );
  }
}