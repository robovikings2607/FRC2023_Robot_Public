package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmPresets;

public class DunkCone extends CommandBase{
    RobotContainer robot;

    public DunkCone(RobotContainer robotContainer) {
        this.robot = robotContainer;
    }

    @Override
    public void initialize() {
        if (robot.intake.getConeMode() && //If we're in cone mode AND
            (robot.arm.getCurrentPreset() == ArmPresets.SCORING_HIGH || //Scoring high OR
            robot.arm.getCurrentPreset() == ArmPresets.SCORING_MID) // Scoring mid
            ) {
            robot.arm.setDunkingCone(true);
            new SetArmPreset(robot, robot.arm.getCurrentPreset()).schedule();
        }
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        robot.arm.setDunkingCone(false);
        new SetArmPreset(robot, robot.arm.getCurrentPreset()).schedule();
    }
}
