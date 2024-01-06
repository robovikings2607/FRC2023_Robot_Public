package frc.robot.commands.AutonCommands.AutonUtilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ResetOdometryFromVision extends CommandBase{
    private RobotContainer robot;
    private Pose2d position;
    
    public ResetOdometryFromVision(RobotContainer robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        var pose = robot.leftCamera.getLastVisionPose();
        if ( pose != null)
        {
            robot.swerveDrive.resetOdometry(pose);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
