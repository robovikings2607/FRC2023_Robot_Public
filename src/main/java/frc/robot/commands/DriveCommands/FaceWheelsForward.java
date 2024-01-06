package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class FaceWheelsForward extends CommandBase{
    private DriveSubsystem swerveDrive;

    public FaceWheelsForward(DriveSubsystem subsystem) {
        swerveDrive = subsystem;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        swerveDrive.faceAllForward(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}