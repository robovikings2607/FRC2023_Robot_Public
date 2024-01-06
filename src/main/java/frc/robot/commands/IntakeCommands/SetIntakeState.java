package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ArmCommands.SetArmPreset;


public class SetIntakeState extends CommandBase{
    private RobotContainer robot;
    private boolean cone;

    public SetIntakeState(RobotContainer robot, boolean cone){
        this.robot = robot;
        this.cone = cone;
        addRequirements(robot.intake);
    }

    @Override
    public void initialize(){
        robot.intake.setSolenoid(cone? 
            IntakeConstants.CONE_GRAB_STATE : !IntakeConstants.CONE_GRAB_STATE);
        robot.intake.setConeMode(cone);
    }
    
    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean i) {
        // Don't extend outside the frame
        //new SetArmPreset(robot, robot.arm.getCurrentPreset()).schedule();
    }
}
