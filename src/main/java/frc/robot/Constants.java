/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
public final class Constants {
  public static double Clamp(double min, double max, double n) {
    return n < min ? min : n > max ? max : n;
  }
  
  public static final class ArmConstants {
    public static final double SHOULDER_NEUTRAL = -0.02; //Gravity constant
    public static final double ELBOW_NEUTRAL = 0.015; //Gravity constant

    public static final double SHOULDER_VELOCITY_FF = 1.5;
    public static final double ELBOW_VELOCITY_FF = 1.4;

    public static final double SHOULDER_ZERO = 0.573; // 0.6867
    public static final double ELBOW_ZERO = 0.82232202;

    //front presets
    //shoulder: .737
    //elbow: .253
    public static final double SHOULDER_STOWING = (SHOULDER_ZERO + (21.96/360.0)) % 1.0;
    public static final double ELBOW_STOWING = (ELBOW_ZERO - 0.55932202) % 1.0;
    //
    public static final double SHOULDER_HYBRID_INTAKING = (SHOULDER_STOWING + 10.0/360.0)% 1.0;
    public static final double ELBOW_HYBRID_INTAKING = ELBOW_STOWING;
    //
    public static final double SHOULDER_HIGH_CONE = (SHOULDER_ZERO - (105.0/360)) % 1.0; 
    public static final double ELBOW_HIGH_CONE = (ELBOW_ZERO + .013678) % 1.0;
    //
    public static final double SHOULDER_HIGH_CUBE = (SHOULDER_ZERO - .173665) % 1.0;
    public static final double ELBOW_HIGH_CUBE = (ELBOW_ZERO - .792322) % 1.0;
    //
    public static final double SHOULDER_MID_CONE = (SHOULDER_ZERO - (90.0/360)) % 1.0; //used to be 87
    public static final double ELBOW_MID_CONE = (ELBOW_ZERO + .047678) % 1.0;
    //
    public static final double COOL_SHOULDER_MID_CONE = (SHOULDER_ZERO - 0.133) % 1.0;
    public static final double COOL_ELBOW_MID_CONE = (ELBOW_ZERO - 0.69562202) % 1.0;
    //
    public static final double SHOULDER_MID_CUBE = (SHOULDER_ZERO - .035665) % 1.0;
    public static final double ELBOW_MID_CUBE = (ELBOW_ZERO - .637322) % 1.0;
    //
    public static final double SHOULDER_INTAKE = (SHOULDER_ZERO - (0.67/360)) % 1.0;  
    public static final double ELBOW_INTAKE = (ELBOW_ZERO -.769322) % 1.0;

    public static final double SHOULDER_HP = (SHOULDER_ZERO - (98.0/360.0)) % 1.0; 
    public static final double ELBOW_HP = ELBOW_HIGH_CONE;
  
    public static final int SHOULDER_MASTER = 15;
    public static final int SHOULDER_SLAVE = 16;
    public static final int SHOULDER_ENCODER_PORT = 9;

    public static final double SHOULDER_P = 3.0;
    public static final double SHOULDER_D = 0;

    public static final int ELBOW_MASTER = 17;
    public static final int ELBOW_SLAVE = 18;
    public static final int ELBOW_ENCODER_PORT = 8;

    public static final double ELBOW_P = 4.0;
    public static final double ELBOW_D = 0.0;

    public static enum ArmPresets {
      SCORING_HIGH,
      SCORING_MID,
      //
      INTAKING,
      HP_INTAKE,
      //
      STOWING,
      //
      HYBRID_INTAKING
    }
  }
  
  public static final class IntakeConstants{
    public static final int INTAKE_MOTOR_ID = 19;
    public static final int INTAKE_SOLENOID = 0;
    public static final int HYBRID_MOTOR_1_ID = 14;  
    public static final int HYBRID_MOTOR_2_ID = 13; 
    public static final int HYBRID_SOLENOID = 7; 
    public static final boolean INTAKE_INVERTED = true;
    public static final boolean CONE_GRAB_STATE = false;
    public static final boolean HYBRID_INTAKE_RETRACTED = false;
    public static final int HAS_PIECE_SENSOR = 7;
    public static final int HAS_PIECE_SENSOR_HEALTHY = 6;
    public static final double CURRENT_MAX = 30.0;
  }
  public static final class VisionConstants {
    public static final Transform3d BOT_TO_LEFT_CAM = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(10), //Forward/back
        Units.inchesToMeters(10.75), //Left/Right
        Units.inchesToMeters(16.5)), //Up/down
      new Rotation3d(0, 0, Math.toRadians(30))
    );
    public static final Transform3d BOT_TO_RIGHT_CAM = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(10), //Forward/back
        Units.inchesToMeters(-10.75), //Left/Right
        Units.inchesToMeters(16.5)), //Up/down
      new Rotation3d(0, 0, Math.toRadians(-30))
    );

    public static final Transform3d LEFT_CAM_TO_BOT = BOT_TO_LEFT_CAM.inverse();
    public static final Transform3d RIGHT_CAM_TO_BOT = BOT_TO_RIGHT_CAM.inverse();

    public static final double AMBIGUITY_THRESHOLD = 0.3;
  }

  public static final class DriveConstants {
    public static int BALANCE_TOLERANCE = 8;
    public static double BALANCE_GOSLOW_RATE = 23;
    public static double BALANCE_kP = 0.011;
    public static double BALANCE_GOSLOW_FACTOR = 0.7;
    
    public static double BALANCE_kD = 0.005;
    public static double BALANCE_MAX_SPEED = 0.8;
    public static double ON_BALANCE_TOLERANCE = 10;
    //starting from right front, then goes counter clockwise. Steer, encoder, then drive.

    //front right module
    public static final int kFrontRightSteerMotorPort = 1;
    public static final int kFrontRightCanCoderPort = 2;
    public static final int kFrontRightFaceForward = 30; //2023
    public static final int kFrontRightDriveMotorPort = 3;

    //front left module
    public static final int kFrontLeftSteerMotorPort = 4;
    public static final int kFrontLeftCanCoderPort = 5;
    public static final int kFrontLeftFaceForward = 156; //2023
    public static final int kFrontLeftDriveMotorPort = 6;

    //back left module
    public static final int kBackLeftSteerMotorPort = 7;
    public static final int kBackLeftCanCoderPort = 8;
    public static final int kBackLeftFaceForward = 97; //2023
    public static final int kBackLeftDriveMotorPort = 9;

    //back right module
    public static final int kBackRightSteerMotorPort = 10;
    public static final int kBackRightCanCoderPort = 11; 
    public static final int kBackRightFaceForward = 19; //2023
    public static final int kBackRightDriveMotorPort = 12;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftTurningMotorReversed = false;
    public static final boolean kRearLeftTurningMotorReversed = false;
    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kRearRightTurningMotorReversed = false;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    //Distance between centers of right and left wheels on robot in meters
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    //Distance between front and back wheels on robot in meters
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2), //FL
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //FR
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //BL
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) //BR
        );

    public static final boolean kGyroReversed = true;

    //for the rotate bot pid
    public static final double rotateBotP = 0.0008;
    public static final double rotateBotI = 0.0;
    public static final double rotateBotD = 0.0;

    public static final double ksVolts = 0.2;
    public static final double kvVoltSecondsPerMeter = 0.5;
    public static final double kaVoltSecondsSquaredPerMeter = 0.1;

    public static final double kMaxSpeedMetersPerSecond = 1.5;

    public static final double omegaScale = 1.0/30.0;

  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 15.0 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12.0 * Math.PI;

    public static final int kMaxDriveSpeedTicksPerSecond = 18000;

    /// One revolution of the relative steering encoder. (Falcon)
    public static final int kEncoderCPRSteer = 26214;

    /// One revolution of the absolute steering encoder. (CANCoder)
    public static final int kAbsoluteEncoderCPRSteer = 4096; //idk

    // 2048 CPR from the motor shaft.
    // 6.75:1 overall reduction on SDS "Fast" gearing.
    public static final double kEncoderCPRDrive = 2048.0 * 6.75;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are mounted on the motor shafts and there are gears between the motor and the wheel
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPRDrive;
    public static final double kDriveTicksPerMeter = 1.0 / kDriveEncoderDistancePerPulse;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPRSteer;//In radians per pulse

    public static final double kPModuleTurningController = 2.0;
    public static final double kPModuleDriveController = 1.0;

    public static final double kDriveMotorPID_P = 0.0;  // Talon SRX was 7.0
    public static final double kDriveMotorPID_I = 0.0;
    public static final double kDriveMotorPID_D = 0.0;
    public static final double kDriveMotorPID_F = 0.05688; //0.05115

    public static final double kTurningMotorPID_P = 0.8;
    public static final double kTurningMotorPID_I = 0.0;
    public static final double kTurningMotorPID_D = 0.0;
    public static final double kTurningMotorPID_F = 0.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0; //2.7 
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; //Math.PI
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI/2; //Math.PI

    public static double kMaxDistanceMetersError = 0.15;
    public static double kMaxAngleDegreesError = 3;
  
    public static double kGoToPointLinearP = 0.35;
    public static double kGoToPointLinearF = 0.2;
    
    public static double kGoToPointAngularP = 0.0009;
    public static double kGoToPointAngularD = 0.00004;
    public static double kGoToPointAngularF = -0.000072;

    public static double maxTrajectoryOverrunSeconds = 0.5;

    //final below
    public static double kPXController = 5.0; //4.3
    public static double kPYController = 5.0; //4.3

    public static double kPThetaController = 3.0; //1.0

    //Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
          kMaxAngularSpeedRadiansPerSecondSquared);

  }

  public static final class LogConstants {
    public static final String logFolderName = "/home/lvuser/pathlogs/";
    public static final String logFileName = "pathLog.csv";
  }

  public static final Pose2d LEFT_BOTTOM_CORNER = new Pose2d(new Translation2d(-8.2423, -4.0513), new Rotation2d());
}
