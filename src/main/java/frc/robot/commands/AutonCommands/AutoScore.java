package frc.robot.commands.AutonCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.commands.ArmCommands.SequentialArmUp;

public class AutoScore extends SequentialCommandGroup{
    private RobotContainer robot;
    Pose2d desiredPose;

    public AutoScore(RobotContainer robot){
        this.robot = robot;
        addCommands(
            new GoToPoint(robot, () -> robot.swerveDrive.getNearestNode().orElse(null)).setAngleTimeOffset(1),
            new SequentialArmUp(robot, ArmPresets.SCORING_HIGH)
        );
    }
}
 