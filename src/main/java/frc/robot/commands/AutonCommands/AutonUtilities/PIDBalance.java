package frc.robot.commands.AutonCommands.AutonUtilities;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDColor;

public class PIDBalance extends CommandBase {
    private DriveSubsystem drive;
    private LED led;
    private Timer steadyTimer, ddxTimer, runtime, stopTime;
    private XboxController controller;
    private PIDController pidController;
    private double lastReading;

    boolean goSlow = false;

    public PIDBalance(RobotContainer robot) {
        this.drive = robot.swerveDrive;
        this.led = robot.led;
        steadyTimer = new Timer();
        ddxTimer = new Timer();
        runtime = new Timer();
        stopTime = new Timer();
        controller = robot.driverController.controller;
        addRequirements(drive);

        pidController = new PIDController(DriveConstants.BALANCE_kP, 0, DriveConstants.BALANCE_kD);
        pidController.setTolerance(DriveConstants.BALANCE_TOLERANCE);
        SmartDashboard.putNumber("BALANCE_kP", DriveConstants.BALANCE_kP);
        SmartDashboard.putNumber("BALANCE_kD", DriveConstants.BALANCE_kD);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Balance/running", true);
        pidController.setPID(SmartDashboard.getNumber("BALANCE_kP", DriveConstants.BALANCE_kP), 0, SmartDashboard.getNumber("BALANCE_kD", DriveConstants.BALANCE_kD));
        controller.setRumble(RumbleType.kBothRumble, 0.5);

        steadyTimer.reset();
        steadyTimer.start();
        ddxTimer.reset();
        ddxTimer.start();
        runtime.start();
        runtime.restart();
        stopTime.start();

        drive.disableRamping();
        lastReading = Math.hypot(drive.getPitch(), drive.getRoll());
        goSlow = false;
    }

    @Override
    public void execute() {
        pidController.setPID(SmartDashboard.getNumber("BALANCE_kP", DriveConstants.BALANCE_kP), 0, SmartDashboard.getNumber("BALANCE_kD", DriveConstants.BALANCE_kD));

        double hypot = Math.hypot(drive.getPitch(), drive.getRoll());
        double ddxVector = (hypot - lastReading)/ddxTimer.get();
        ddxTimer.reset();
        lastReading = hypot;

        double pitchPercent = drive.getPitch() / hypot;
        double rollPercent = drive.getRoll() / hypot;

        var power = pidController.calculate(hypot, 0);

        if (!pidController.atSetpoint()) {
            steadyTimer.reset();
        }
        else {
            goSlow = true;
        }

        if ( ddxVector < -DriveConstants.BALANCE_GOSLOW_RATE && runtime.hasElapsed(0.5)) {
            goSlow = true;
        }


        if ( goSlow ) {
            power *= DriveConstants.BALANCE_GOSLOW_FACTOR;
        }

        if ( pidController.atSetpoint() ) {
            power = 0;
        }

        double forwardPower = Constants.Clamp(-DriveConstants.BALANCE_MAX_SPEED, DriveConstants.BALANCE_MAX_SPEED, pitchPercent * power);
        double strafePower = Constants.Clamp(-DriveConstants.BALANCE_MAX_SPEED, DriveConstants.BALANCE_MAX_SPEED, rollPercent * power);
        drive.swerveDrive(strafePower, forwardPower, 0, 1);

        SmartDashboard.putBoolean("Balance/atSetpoint", pidController.atSetpoint());
        SmartDashboard.putBoolean("Balance/goSlow", goSlow);
        SmartDashboard.putNumber("Balance/hypot", hypot);
        SmartDashboard.putNumber("Balance/hypotRate", ddxVector);
        SmartDashboard.putNumber("Balance/pitchPercent", pitchPercent);
        SmartDashboard.putNumber("Balance/rollPercent", rollPercent);
        SmartDashboard.putNumber("Balance/output", power);
    }

    @Override
    public boolean isFinished() {
        return (
            steadyTimer.get() >= 3 ||
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
        SmartDashboard.putBoolean("Balance/running", false);
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