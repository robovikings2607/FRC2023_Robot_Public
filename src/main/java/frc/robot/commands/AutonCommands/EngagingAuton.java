// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.commands.ArmCommands.SetArmPreset;
import frc.robot.commands.AutonCommands.AutonUtilities.DriveOnChargeStation;
import frc.robot.commands.AutonCommands.AutonUtilities.PIDBalance;
import frc.robot.commands.AutonCommands.AutonUtilities.ResetOdometry;
import frc.robot.commands.IntakeCommands.Drop;
import frc.robot.commands.IntakeCommands.SetIntakeState;
import frc.robot.subsystems.DriveSubsystem;


public class EngagingAuton extends SequentialCommandGroup {

  private DriveSubsystem driveSubsystem;
  private int allianceSign;
  Rotation2d startingRot;
  // private static DriverStation DS;

  public EngagingAuton(RobotContainer robot) {
    driveSubsystem = robot.swerveDrive;
    addRequirements(driveSubsystem);

    if (DriverStation.getAlliance() == Alliance.Invalid) {
      System.out.println("ALLIANCE INVALID");
      return; //Maybe do something later
    }

    allianceSign = (DriverStation.getAlliance() == Alliance.Blue) ? -1 : 1;

    startingRot = new Rotation2d((Math.PI/2) - (allianceSign*(Math.PI/2)));

    addCommands(             // to add: go over charging station and come back
      new ResetOdometry(
        driveSubsystem, 
        new Pose2d(new Translation2d((6.43-Units.inchesToMeters(20.0))*allianceSign, -1.28), //OG value 6.43
        startingRot)),
      new SetIntakeState(robot, false),
      new SetArmPreset(robot, ArmPresets.SCORING_HIGH).yield(),
      new Drop(robot),
      new SetArmPreset(robot, ArmPresets.STOWING),
      new DriveOnChargeStation(robot, -0.7),
      new PIDBalance(robot)
    );
  }
}