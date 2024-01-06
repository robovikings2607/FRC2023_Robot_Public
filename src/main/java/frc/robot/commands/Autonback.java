package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Autonback extends CommandBase {@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveSubsystem drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Autonback(DriveSubsystem subsystem) {
   drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  private Timer autontimer = new Timer();
  private boolean isDone;

  @Override
  public void initialize() {
      autontimer.stop();
      autontimer.reset();
      autontimer.start();
      isDone = false;
  }

  @Override
  public void execute() {
    if (autontimer.get() > 0.0 && autontimer.get() < 0.6) {
       drive.swerveDrive(0.0, 0.0, 0.25, 1);
    } else if (autontimer.get() > 0.6 && autontimer.get() < 1.6){
        drive.swerveDrive(0.0, 0.0, 0.0, 1);
  } else if (autontimer.get() > 1.6 && autontimer.get() < 2.1) {
      drive.swerveDrive(0.0, 0.25, 0.0, 1);
  } else if (autontimer.get() > 2.2) {
      drive.swerveDrive(0.0, 0.0, 0.0, 1);
      isDone = true;
  }
}



  @Override
  public boolean isFinished() {
    return isDone;
  }
}

