/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.commands.ArmCommands.SetArmPreset;
import frc.robot.commands.IntakeCommands.ZeroIntake;
import frc.robot.utilities.REVChecker;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
 // private Relay LEDRelay = new Relay(0, Direction.kBoth);

  DashboardUpdater<Constants.AutoConstants> autoConstantsUpdater = new DashboardUpdater<>(AutoConstants.class, "AutoConstants");
  DashboardUpdater<Constants.DriveConstants> driveDashboardUpdater = new DashboardUpdater<>(DriveConstants.class, "DriveConstants");

  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();

    // Start logging
    if ( Robot.isReal() )
    {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }

    DriverStation.silenceJoystickConnectionWarning(true);
    m_robotContainer = new RobotContainer();

    SmartDashboard.putBoolean("ReadyToRun", !REVChecker.hadError());
  }

  @Override
  public void robotPeriodic() {
    double startTime = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    double execTime = Timer.getFPGATimestamp() - startTime;
    SmartDashboard.putNumber("robotPeriodicTimeMS", execTime * 1000.0);
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.swerveDrive.setTurnMotors(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("headingTarget", 0);
    SmartDashboard.putNumber("headingCurrent", 0);
    SmartDashboard.putBoolean("isDriveCalibrated", m_robotContainer.swerveDrive.isAllCalibrated());
  }

  @Override
  public void autonomousInit() {
    Timer.delay(0.25);

    //m_robotContainer.intake.setDefaultCommand(new RepeatCommand(new IntakeCommand(m_robotContainer)));

    m_robotContainer.swerveDrive.faceAllForward(true);
    m_autonomousCommand = (m_robotContainer.getAutonCommand() != null) ? m_robotContainer.getAutonCommand() : new WaitCommand(15);
    m_autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //
    m_robotContainer.swerveDrive.faceAllForward(true);

    m_robotContainer.intake.setDefaultCommand(new ZeroIntake(m_robotContainer));


    m_robotContainer.arm.setCurrentPreset(ArmPresets.INTAKING);
    new SetArmPreset(m_robotContainer, ArmPresets.STOWING).schedule();
    m_robotContainer.setFieldCentric(true);
    // m_robotContainer.arm.shoulderPIDController.reset(m_robotContainer.arm.shoulderEncoder.getAbsolutePosition());
    // m_robotContainer.arm.elbowPIDController.reset(m_robotContainer.arm.elbowEncoder.getAbsolutePosition());
    }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.arm.simulationPeriodic();
  }
}