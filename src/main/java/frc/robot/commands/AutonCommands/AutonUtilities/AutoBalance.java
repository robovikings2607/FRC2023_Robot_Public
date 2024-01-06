package frc.robot.commands.AutonCommands.AutonUtilities;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDColor;

public class AutoBalance extends CommandBase {
    private DriveSubsystem drive;
    private LED led;
    private Timer steadyTimer, ddxTimer;
    private XboxController controller;
    private PIDController pidController;
    private double lastReading;

    private final double P, I, D;

    public AutoBalance(RobotContainer robot) {
        P = 0.015;
        I = 0;
        D = 0;
        this.drive = robot.swerveDrive;
        this.led = robot.led;
        steadyTimer = new Timer();
        ddxTimer = new Timer();
        controller = robot.driverController.controller;
        addRequirements(drive);

        pidController = new PIDController(P, I, D);
    }

    @Override
    public void initialize() {
        controller.setRumble(RumbleType.kBothRumble, 0.5);

        steadyTimer.stop();
        steadyTimer.reset();
        ddxTimer.stop();
        ddxTimer.reset();
        ddxTimer.start();

        drive.disableRamping();
        lastReading = Math.hypot(drive.getPitch(), drive.getRoll());
    }

    @Override
    public void execute() {
        double hypot = Math.hypot(drive.getPitch(), drive.getRoll());
        double ddxVector = (hypot - lastReading)/ddxTimer.get();
        lastReading = hypot;

        if (ddxVector < 0.0 || hypot <= DriveConstants.BALANCE_TOLERANCE){
            steadyTimer.restart(); //We're balancing
            pidController.setP(P * Math.sin(Math.toRadians(hypot)));
            // pidController.setP(0);
        } else if (ddxVector > 0.0 || hypot >= DriveConstants.BALANCE_TOLERANCE){
            pidController.setP(P);
            steadyTimer.stop();
            steadyTimer.reset();
        }


        var strafePID = Constants.Clamp(-1, 1, pidController.calculate(drive.getRoll()));
        var forwardPID = Constants.Clamp(-1, 1, pidController.calculate(drive.getPitch()));
        drive.swerveDrive(strafePID, forwardPID, 0, 1);
        
        ddxTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return (
            steadyTimer.get() >= 1.5 ||
            Math.hypot(controller.getLeftX(), controller.getLeftY()) > 0.1 ||
            Math.hypot(controller.getRightX(), controller.getRightY()) > 0.1
        );
    }

    @Override
    public void end(boolean interupted) {
        drive.stopDriveMotors(ControlMode.Velocity);
        drive.enableRamping();
        ddxTimer.stop();
        controller.setRumble(RumbleType.kBothRumble, 0);
        led.setLED(LEDColor.GREEN);
    }
}




// if (Math.hypot(drive.getPitch(), drive.getRoll()) > DriveConstants.BALANCE_TOLERANCE) {
    //         var strafePID = Constants.Clamp(-1, 1, pidController.calculate(drive.getRoll()));
    //         var forwardPID = Constants.Clamp(-1, 1, pidController.calculate(drive.getPitch()));
    //         drive.swerveDrive(strafePID, forwardPID, 0, 1);
    //         System.out.println("Strafe "+strafePID+". Forward "+forwardPID);
    //     } else if (steadyTimer.get() == 0) {
    //         steadyTimer.start();
    //         drive.stopDriveMotors(ControlMode.Velocity);
    //         return;
    //     }