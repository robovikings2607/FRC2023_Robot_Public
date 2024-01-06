package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HybridIntakeSubsystem;

public class HybridIntakeCommand extends CommandBase{
    HybridIntakeSubsystem intake;

    public HybridIntakeCommand(RobotContainer robot){
        intake = robot.hybridIntake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.setHybridIntake(0.5);
    }

    @Override
    public void end(boolean i){
        intake.setHybridIntake(0.0);
    }

    @Override
    public boolean isFinished(){
        return false/*intake.hasGamePiece()*/;
    }
}