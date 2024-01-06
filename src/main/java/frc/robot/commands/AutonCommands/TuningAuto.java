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


public class TuningAuto extends SequentialCommandGroup {

  private DriveSubsystem driveSubsystem;
  private int allianceSign;
  Rotation2d startingRot;


  static final TrapezoidProfile.Constraints SPEEDY_CONSTRAINTS = new TrapezoidProfile.Constraints(1.8, 2);
  static final TrapezoidProfile.Constraints SLOWER_DECEL_CONSTRAINTS = new TrapezoidProfile.Constraints(1.8, 1.8);
  
  public TuningAuto(RobotContainer robot) {
    driveSubsystem = robot.swerveDrive;
    addRequirements(driveSubsystem);

    allianceSign = (DriverStation.getAlliance() == Alliance.Blue) ? -1 : 1;

    startingRot = new Rotation2d((Math.PI/2) - (allianceSign*(Math.PI/2)));

    Translation2d startPose = new Translation2d((6.43-Units.inchesToMeters(20.0))*allianceSign, 0.31);



    addCommands(
      new ResetOdometry(
        driveSubsystem, 
        new Pose2d(new Translation2d(), new Rotation2d())),
      new GoToPoint(robot, new Translation2d(Units.feetToMeters(3), 0), new Rotation2d()).withLinearConstraints(SPEEDY_CONSTRAINTS)
    );
  }
}
