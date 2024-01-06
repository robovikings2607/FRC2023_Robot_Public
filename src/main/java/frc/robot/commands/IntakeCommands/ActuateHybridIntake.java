package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ArmCommands.SetArmPreset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HybridIntakeSubsystem;

public class ActuateHybridIntake extends CommandBase{
    
    HybridIntakeSubsystem intake;
    boolean state;
    ArmSubsystem arm;

    public ActuateHybridIntake(RobotContainer robot, boolean intakeState){
        intake = robot.hybridIntake;
        state = intakeState;
        arm = robot.arm;
        addRequirements(intake, arm);
    }

    @Override
    public void initialize(){
            intake.setHybridSolenoidState(state);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
