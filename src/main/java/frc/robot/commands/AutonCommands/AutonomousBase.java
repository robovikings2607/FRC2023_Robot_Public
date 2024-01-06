package frc.robot.commands.AutonCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonCommands.AutonUtilities.ResetOdometry;
import frc.robot.commands.DriveCommands.FaceWheelsForward;

public class AutonomousBase extends SequentialCommandGroup {
    protected RobotContainer robot;

    public AutonomousBase(RobotContainer robot, Pose2d initialPosition)
    {
        this.robot = robot;
        addRequirements(robot.swerveDrive);

        addCommands(
            new FaceWheelsForward(robot.swerveDrive),
            new WaitCommand(1),
            new ResetOdometry(robot.swerveDrive, initialPosition)
        );
    }


}