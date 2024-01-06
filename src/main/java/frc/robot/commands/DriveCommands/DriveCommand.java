
package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotContainer;

public class DriveCommand extends CommandBase {@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DriveSubsystem driveSubsystem;
  private RobotContainer robotContainer;

  private static final boolean debug=true;  // Turn debug on/off to display variables on Shuffleboard

  private final OI m_driverController; 
  public static final double SLOWGEAR = 0.25;  // The maximum speed for slow gear relative to the maximum speed of the robot
  public static final double HIGHGEAR = 1.0;   // The maximum speed for high gear relative to the maximum speed of the robot
  public static final double OMEGA_SCALE = 0.03;  // The speed of rotation relative to the maximum speed of rotation
	public static final double DEADZONE = 0.05;  // The deadzone for the remote control as a fraction of the joystick travel
  public static final double DEADZONECORRECTION = 1 / (1 - DEADZONE);  // Correction factor for the deadzone for smooth low speed operation

	private double originHeading = -180.0; // The heading of the field relative to the driver
  private double strafe, forward, omega;  // Values for werve drive directions
  private boolean highGear = true;  // High or low gear indicator

  private double leftX, leftY, rightX;  // Xbox controller inputs

  public DriveCommand(RobotContainer robot) {
    robotContainer = robot;
    m_driverController = robotContainer.driverController;
    driveSubsystem = robotContainer.swerveDrive;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.enableRamping();
    driveSubsystem.faceAllForward(false);
    SmartDashboard.putBoolean("Low Gear?", !highGear);
  }

  @Override
  public void execute() { 
    
    if (robotContainer.driverController.controller.getXButtonPressed()) {
      
      robotContainer.toggleDriveMode();
    }
   
    double navHeading = driveSubsystem.getnavxFusedHeading();  // Current robot heading

    // Step 1: Get inputs from the Xbox controller

    // Button Y pressed: Reset the field orientation to make it the same as the robot orientation at the moment this button is pressed
    if (m_driverController.controller.getYButtonPressed()) {  
      originHeading = navHeading;
    }


    // Right stick pressed: Toggle bewteen high gear and low gear 
    if (m_driverController.controller.getRightStickButtonPressed())
    {
      highGear = !highGear;
      SmartDashboard.putBoolean("Low Gear?", !highGear);
    }

    // Get the inputs from the controller joysticks
    leftX = m_driverController.controller.getLeftX(); // Left stick X axis: Right or Left motion request
    leftY = -m_driverController.controller.getLeftY(); //Left stick Y axis: Forward or Reverse motion request.  Nocice the minus sign to correct for stick inversion.
    rightX = m_driverController.controller.getRightX(); // Right stick X axis: Clockwise (CW) or Counterclockwise (CCW) motion request


    // Step 2. Process the joystick inputs and calculate swerve inputs forward, strafe, and omega.
    // In this step, the deadzone is applied and the deadzone correction is also applied.
    if (Math.abs(leftX) < DEADZONE) {strafe = 0;} 
    else {strafe = (Math.abs(leftX) - DEADZONE) * Math.signum(leftX) * DEADZONECORRECTION;}

    if (Math.abs(leftY) < DEADZONE) {forward = 0;} 
    else {forward = (Math.abs(leftY) - DEADZONE) * Math.signum(leftY) * DEADZONECORRECTION;;}

    if (Math.abs(rightX) < DEADZONE) {omega = 0;} 
    else {omega = (Math.abs(rightX) - DEADZONE) * Math.signum(rightX) * DEADZONECORRECTION * OMEGA_SCALE;}

    // If the robot is in field centric mode, then calculate forward and strafe for field centric movement.
    // Otherwise, keep the values for foward and strafe that were calculated before (robot centric mode).
    
    if (robotContainer.getFieldCentric()) {
      // Step 3.  Translate the forward and strafe values for field centric motion.
      // Without these translations, the robot will move as in robot centric mode.

      // Calculate the difference between the robot heading and and the field heading
      double originCorrection = Math.toRadians(originHeading - navHeading);

      // This is an intermediate calculation simply because we cannot change the value of the variable
      // "forward" until after the strafe is calculated.  So "temp" hold the field centric version of
      // forward.
      double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);

      // Calculates the field centric translation for strafe.
      strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);

      // Enter the field centric version for forward
      forward = temp;
    }

    
    //Step 4.  Select the factor for driving at slow speed and high speed
    double driveSpeedScale = (highGear ? HIGHGEAR : SLOWGEAR);


    // Step 5.  Send the command to the swerve drive using the calculated values for strafe, forward, omega, and low/high speed scale
    driveSubsystem.swerveDrive(strafe, forward, omega, driveSpeedScale);

    if (debug)
    {
      SmartDashboard.putNumber("OriginHeading", originHeading);
      SmartDashboard.putNumber("NavHeading", navHeading);
      // These are the inputs from the Xbox controller sticks
      SmartDashboard.putNumber("leftX", leftX);
      SmartDashboard.putNumber("leftY", leftY);
      SmartDashboard.putNumber("rightX", rightX);
      // Inputs of the Xbox controller sticks after deadzone processing
      SmartDashboard.putNumber("leftX_Corrected", leftX);
      SmartDashboard.putNumber("leftY_Corrected", leftY);
      SmartDashboard.putNumber("rightX_Corrected", rightX);
      // Values of the swerve module inputs
      SmartDashboard.putNumber("Strafe", strafe);
      SmartDashboard.putNumber("Foward", forward);
      SmartDashboard.putNumber("Omega", omega);
      SmartDashboard.putNumber("DriveScale", driveSpeedScale);  
    }
   }
  
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.disableRamping();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}