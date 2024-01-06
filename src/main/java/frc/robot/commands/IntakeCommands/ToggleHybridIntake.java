package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HybridIntakeSubsystem;

public class ToggleHybridIntake extends CommandBase{
    
    HybridIntakeSubsystem intake;

    public ToggleHybridIntake(RobotContainer robot){
        intake = robot.hybridIntake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setHybridSolenoidState(!intake.getHybridIntakeState());
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
