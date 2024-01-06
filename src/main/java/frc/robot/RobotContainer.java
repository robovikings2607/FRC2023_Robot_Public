package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ArmConstants.ArmPresets; //syncronous interrupt
import frc.robot.commands.ArmCommands.DunkCone;
import frc.robot.commands.ArmCommands.MidNodeWrapper;
import frc.robot.commands.ArmCommands.SequentialArmUp;
import frc.robot.commands.ArmCommands.SetArmPreset;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HybridIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.AutonCommands.AutoScore;
import frc.robot.commands.AutonCommands.EngageMobilityCone;
import frc.robot.commands.AutonCommands.EngagingAuton;
import frc.robot.commands.AutonCommands.EngagingMobilityAuton;
import frc.robot.commands.AutonCommands.ScoringMobilityAuton;
import frc.robot.commands.AutonCommands.ThreePieceNonCable;
import frc.robot.commands.AutonCommands.ThreePieceStartCone;
import frc.robot.commands.AutonCommands.TuningAuto;
import frc.robot.commands.AutonCommands.TwoPiece;
import frc.robot.commands.AutonCommands.TwoPieceCableProtector;
import frc.robot.commands.DriveCommands.DriveCommand;
import frc.robot.commands.AutonCommands.AutonUtilities.AutoChooser;
import frc.robot.commands.AutonCommands.AutonUtilities.PIDBalance;
import frc.robot.commands.AutonCommands.AutonUtilities.ResetOdometryFromVision;
import frc.robot.commands.IntakeCommands.ActuateHybridIntake;
import frc.robot.commands.IntakeCommands.Drop;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.HybridIntakeCommand;
import frc.robot.commands.IntakeCommands.HybridOuttake;
import frc.robot.commands.IntakeCommands.SwitchIntakeType;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PiCameraSubsystem;

public class RobotContainer {
  public Field2d field = new Field2d();

  // The robot's controller class
  public OI driverController = new OI(OIConstants.kDriverControllerPort);
  public OI operatorController = new OI(OIConstants.kOperatorControllerPort);
  // The robot's subsystems
  public final DriveSubsystem swerveDrive = new DriveSubsystem(this);
  public final PiCameraSubsystem leftCamera = 
    new PiCameraSubsystem(this, "LeftGlobalShutter", VisionConstants.BOT_TO_LEFT_CAM);
  public final PiCameraSubsystem rightCamera = 
    new PiCameraSubsystem(this, "RightGlobalShutter", VisionConstants.BOT_TO_RIGHT_CAM);
  public final LED led = new LED();

  public final ArmSubsystem arm = new ArmSubsystem();
  

  public final IntakeSubsystem intake = new IntakeSubsystem(this);
  public final HybridIntakeSubsystem hybridIntake = new HybridIntakeSubsystem();
  // The robot's commands
  private final DriveCommand driveCommand = new DriveCommand(this);
  
  private AutoChooser autonChooser = new AutoChooser();

  public RobotContainer() {
    autonChooser.addOption("DoNothing", () -> new WaitCommand(1));
    // autonChooser.addOption("EngagingAuton", () -> new EngagingAuton(this));
    autonChooser.addOption("EngageMobilityCube", () -> new EngagingMobilityAuton(this));
    autonChooser.addOption("EngageMobilityCone", () -> new EngageMobilityCone(this));

    autonChooser.addOption("OnePieceMobility", () -> new ScoringMobilityAuton(this));
    // autonChooser.addOption("TwoPiece(No cable)", () -> new TwoPiece(this));
    autonChooser.addOption("CONE_YELLOW_TWO PIECE CABLE SIDE", () -> new TwoPieceCableProtector(this));
    autonChooser.addOption("ThreePieceCUBE_PURPLE", () -> new ThreePieceNonCable(this));
    autonChooser.addOption("CONE_YELLOWThreePiece", () -> new ThreePieceStartCone(this));
    autonChooser.addOption("[DONT USE]TuningAuto", () -> new TuningAuto(this));

    SmartDashboard.putData("AutonChooser", autonChooser);
    SmartDashboard.putData("Field!!!", field);

    configureButtonBindings();

    swerveDrive.setDefaultCommand(driveCommand);
    setFieldCentric(true);
  }


  private boolean fieldCentricDrive;

  public void setFieldCentric(boolean state)
  {
    fieldCentricDrive = state;
    SmartDashboard.putBoolean("FieldCentricMode", fieldCentricDrive);
  }

  public boolean getFieldCentric() {
    return fieldCentricDrive;
  }
  public void toggleDriveMode() {
    setFieldCentric(!fieldCentricDrive);
  } // toggleDriveMode

  private void configureButtonBindings() {
    //DRIVER:
    driverController.buttonA.onTrue(new PIDBalance(this));
    driverController.leftBumper.onTrue(new Drop(this));
    driverController.startButton.onTrue (new ResetOdometryFromVision(this));
    // driverController.leftBumper.onTrue(new DunkCone(this));
    driverController.rightBumper.whileTrue(new DunkCone(this));
    driverController.buttonB.whileTrue(new AutoScore(this));
    driverController.leftTriggerButton.whileTrue(new HybridOuttake(this, false));
    driverController.rightTriggerButton.whileTrue(new HybridOuttake(this, true));


    //OPERATOR:
    operatorController.buttonY.onTrue(new SetArmPreset(this, ArmPresets.SCORING_HIGH));
    operatorController.buttonB.onTrue(new SetArmPreset(this, ArmPresets.INTAKING));
    operatorController.buttonA.onTrue(new SetArmPreset(this, ArmPresets.STOWING));
    operatorController.buttonX.onTrue(new MidNodeWrapper(this, ArmPresets.SCORING_MID));
    //
    operatorController.backButton.onTrue(new SetArmPreset(this, ArmPresets.HP_INTAKE));
    operatorController.startButton.onTrue(new SequentialArmUp(this, ArmPresets.SCORING_HIGH));
    //
    operatorController.rightTriggerButton.whileTrue(new RepeatCommand(new IntakeCommand(this)));
    operatorController.rightBumper.onTrue(new SwitchIntakeType(this));
    //
    operatorController.povNorth.onTrue(new SetArmPreset(this, ArmPresets.STOWING)
        .andThen(new ActuateHybridIntake(this, IntakeConstants.HYBRID_INTAKE_RETRACTED)));
    operatorController.povSouth.onTrue(new ActuateHybridIntake(this, !IntakeConstants.HYBRID_INTAKE_RETRACTED)
        .andThen(new SetArmPreset(this, ArmPresets.HYBRID_INTAKING)));
    //
    operatorController.leftTriggerButton.whileTrue(new RepeatCommand(
      new HybridIntakeCommand(this)
    ));
  }

  public Command getAutonCommand() {
    Command auto = autonChooser.getSelected();
    if ( auto != null )
    {
      System.out.println("SELECTED auto is "+ auto.getName());
    }
    else {
      System.out.println("NULL AUTO");
    }
    return autonChooser.getSelected();
  }

}