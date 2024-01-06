package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ZeroIntake extends CommandBase {
    RobotContainer robot;
    public ZeroIntake(RobotContainer robot)
    {
        this.robot = robot;
        addRequirements(robot.intake);
    }

    @Override
    public void initialize() {
        robot.intake.setMotor(0);
    }
}
