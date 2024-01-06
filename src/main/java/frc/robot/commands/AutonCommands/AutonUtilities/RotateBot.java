package frc.robot.commands.AutonCommands.AutonUtilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RotateBot extends CommandBase{
    private DriveSubsystem swerve;
    private double deltaDegrees, goal, omega;
    private PIDController pid;
    private int consistencyCount = 0;
    private Timer timer;

    public RotateBot(DriveSubsystem driveSystem, double degrees) {
        swerve = driveSystem;
        addRequirements(swerve);
        deltaDegrees = (degrees > 180) ? 180 : (degrees < -180) ? -180 : degrees;
        pid = new PIDController(DriveConstants.rotateBotP, DriveConstants.rotateBotI, DriveConstants.rotateBotD);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        goal = (swerve.getHeadingDegrees() + deltaDegrees);
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        omega = -pid.calculate(swerve.getHeadingDegrees(), goal);
        swerve.swerveDrive(0.0, 0.0, omega, 1.0);

        if (Math.abs(swerve.getHeadingDegrees() - goal) < 1) {
            consistencyCount++;
        } else {
            consistencyCount = 0;
        }
    }

    @Override
    public boolean isFinished() { // || doesn't want to work 
        if (timer.get() > 5.0) {
            return true;
        }
        return consistencyCount > 30;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("RotateBotOver", true);
        timer.stop();
        timer.reset();
    }
}