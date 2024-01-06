package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ArmCommands.SetArmPreset;
import frc.robot.subsystems.LED.LEDColor;

public class SwitchIntakeType extends CommandBase{
    private RobotContainer robot;

    public SwitchIntakeType(RobotContainer robot) {
        this.robot = robot;
        addRequirements(robot.intake);
    }
    
    @Override
    public void initialize() {
        robot.intake.setConeMode(!robot.intake.getConeMode());
        if (robot.intake.getConeMode()) {
            robot.led.setLED(LEDColor.YELLOW);
        } else {
            robot.led.setLED(LEDColor.PURPLE);
        }

        robot.intake.setSolenoid(robot.intake.getConeMode() ? 
            IntakeConstants.CONE_GRAB_STATE : !IntakeConstants.CONE_GRAB_STATE);
        }
       

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean i) {
        new SetArmPreset(robot, robot.arm.getCurrentPreset()).schedule();
    }
}