package frc.robot.commands.AutonCommands.AutonUtilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometry extends CommandBase{
    private DriveSubsystem driveSubsystem;
    private Pose2d position;
    
    public ResetOdometry(DriveSubsystem drive, Pose2d pos) {
        driveSubsystem = drive;
        position = pos;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.zeroHeading();
        driveSubsystem.resetOdometry(position);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
