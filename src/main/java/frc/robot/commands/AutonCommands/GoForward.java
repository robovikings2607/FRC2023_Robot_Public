package frc.robot.commands.AutonCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonCommands.AutonUtilities.ResetOdometry;

public class GoForward extends SequentialCommandGroup {
    public GoForward(RobotContainer robot)
    {
        addCommands(
            new ResetOdometry(robot.swerveDrive, new Pose2d(new Translation2d(-5.9, -1.28), Rotation2d.fromDegrees(180))),
            new GoToPoint(robot, new Translation2d(-3.1, -1.28), Rotation2d.fromDegrees(180))
        );
    }
    
}
