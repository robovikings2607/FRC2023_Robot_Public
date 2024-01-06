package frc.robot.commands.IntakeCommands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDColor;

public class IntakeCommand extends CommandBase{
    
    private IntakeSubsystem intake;
    private LED led;
    private LEDColor gamePieceColor;
    private boolean hasPiece, runningAutomatic;
    private double timeout;
    private Timer timeoutTimer;

    public IntakeCommand(RobotContainer robot){
        intake = robot.intake;
        led = robot.led;

        addRequirements(intake);
        timeoutTimer = new Timer();
        timeout = 0;
        runningAutomatic = DriverStation.isAutonomousEnabled();
    } 

    


    @Override
    public void initialize() {
        gamePieceColor = intake.getConeMode() ? LEDColor.YELLOW : LEDColor.PURPLE;
        led.setLED(gamePieceColor);
        if (runningAutomatic){
            timeoutTimer.restart();
        }
    }
    
    public void execute(){
        hasPiece = intake.hasPiece();

        if (intake.getConeMode() ) {
            intake.setSolenoid(IntakeConstants.CONE_GRAB_STATE);
            intake.setMotor(hasPiece ? 0 : 1);
            gamePieceColor = LEDColor.YELLOW;
        } else {
            intake.setSolenoid(!IntakeConstants.CONE_GRAB_STATE);
            intake.setMotor(hasPiece ? 0.3 : 1); 
            gamePieceColor = LEDColor.PURPLE;
        }
    }

    @Override
    public void end(boolean interrupted){
        // if (hasPiece){
        //    // led.setLED(gamePieceColor, true, 6);
        // } else {
        //    // led.setLED(gamePieceColor, false, 0);
        // }
    }

    @Override
    public boolean isFinished(){
        return (hasPiece) ||
            (runningAutomatic && timeoutTimer.get() >= timeout); //Holding a piece
 
    }
    public IntakeCommand timeout(double Timeout) {
        this.timeout = Timeout;
        this.runningAutomatic = true;
        return this;
    }
}
