package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HybridIntakeSubsystem;

public class HybridOuttake extends CommandBase{

    HybridIntakeSubsystem intake;
    boolean fastMode;
    double speed;
    Timer timeoutTimer;

    public HybridOuttake(RobotContainer robot, boolean fastMode){
        intake = robot.hybridIntake;
        this.fastMode = fastMode;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        speed = fastMode ? -1.0: -0.4;
        intake.setHybridIntake(speed);
    }

    @Override
    public void end(boolean i){
        intake.setHybridIntake(0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}