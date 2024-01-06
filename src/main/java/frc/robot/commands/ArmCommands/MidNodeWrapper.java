package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmPresets;

public class MidNodeWrapper extends CommandBase{
    private ArmPresets preset;
    private RobotContainer robot;

    public MidNodeWrapper(RobotContainer robot, ArmPresets preset) {
        this.robot = robot;
        this.preset = preset;
    }

    @Override
    public void initialize()
    {
        if (robot.intake.getConeMode()){
            new SequentialArmUp(robot, preset).schedule();
        } else {
            new SetArmPreset(robot, preset).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
