package frc.robot.commands.AutonCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.commands.ArmCommands.SetArmPreset;
import frc.robot.commands.AutonCommands.AutonUtilities.ResetOdometry;
import frc.robot.commands.IntakeCommands.Drop;
import frc.robot.commands.IntakeCommands.SetIntakeState;
import frc.robot.subsystems.DriveSubsystem;

public class ScoringMobilityAuton extends SequentialCommandGroup {
    
    DriveSubsystem driveSubsystem;
    int allianceSign;
    Rotation2d startingRot;

    public ScoringMobilityAuton(RobotContainer robot){
        driveSubsystem = robot.swerveDrive;
        addRequirements(driveSubsystem);

        allianceSign = (DriverStation.getAlliance() == Alliance.Blue) ? -1 :1;

        startingRot = new Rotation2d((Math.PI/2)-(allianceSign*(Math.PI/2)));

        addCommands(
            new ResetOdometry(
                driveSubsystem, 
                new Pose2d(new Translation2d((6.43-Units.inchesToMeters(20.0))*allianceSign, -1.28), //OG value 6.43
                    startingRot)),
            new SetIntakeState(robot, false),
            new SetArmPreset(robot, ArmPresets.SCORING_HIGH).yield(),
            new Drop(robot),
            new ParallelCommandGroup(
                new GoToPoint(
                    robot,
                    new Translation2d(1.5*allianceSign, -1.28),
                    startingRot)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new SetArmPreset(robot, ArmPresets.STOWING)
                )
        );

    }

    
}
