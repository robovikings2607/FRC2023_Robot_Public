package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class Drop extends CommandBase{
    private IntakeSubsystem intake;
    private boolean coneMode;
    private Timer dropTimer;

    public Drop(RobotContainer robot) {
        intake = robot.intake;
        dropTimer = new Timer();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        coneMode = intake.getConeMode();
        if (coneMode) {
            intake.setSolenoid(!IntakeConstants.CONE_GRAB_STATE);
        } else {
            intake.setMotor(-.30);
            dropTimer.reset();
            dropTimer.start();
        }
    }

    @Override
    public boolean isFinished() {
        // System.out.println("is cone mode " + coneMode);
        return coneMode || 
            //intake.getGamePieceProximity() <= 60 && intake.getGamePieceProximity() != 0 || 
        //   !intake.hasPiece()||
            dropTimer.get() >= 0.5;
    }

    @Override
    public void end(boolean i) {
        intake.setMotor(0);
        dropTimer.stop();
    }

}