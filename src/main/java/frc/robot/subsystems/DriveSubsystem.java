/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;


import java.util.Arrays;
import java.util.Collections;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.sim.OdomSimulation;
import frc.robot.utilities.RVAprilTagField;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  //Robot swerve modules
  public SwerveModule m_frontLeft
      = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
                         DriveConstants.kFrontLeftSteerMotorPort,
                         DriveConstants.kFrontLeftCanCoderPort,
                         DriveConstants.kFrontLeftDriveEncoderReversed,
                         DriveConstants.kFrontLeftTurningEncoderReversed,
                         DriveConstants.kFrontLeftDriveMotorReversed,
                         DriveConstants.kFrontLeftTurningMotorReversed,
                         DriveConstants.kFrontLeftFaceForward);

  public SwerveModule m_backLeft =
      new SwerveModule(DriveConstants.kBackLeftDriveMotorPort,
                       DriveConstants.kBackLeftSteerMotorPort,
                       DriveConstants.kBackLeftCanCoderPort,
                       DriveConstants.kRearLeftDriveEncoderReversed,
                       DriveConstants.kRearLeftTurningEncoderReversed,
                       DriveConstants.kRearLeftDriveMotorReversed,
                       DriveConstants.kRearLeftTurningMotorReversed,
                       DriveConstants.kBackLeftFaceForward);

  public SwerveModule m_frontRight =
      new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
                       DriveConstants.kFrontRightSteerMotorPort,
                       DriveConstants.kFrontRightCanCoderPort,
                       DriveConstants.kFrontRightDriveEncoderReversed,
                       DriveConstants.kFrontRightTurningEncoderReversed,
                       DriveConstants.kFrontRightDriveMotorReversed,
                       DriveConstants.kFrontRightTurningEncoderReversed,
                       DriveConstants.kFrontRightFaceForward);

  public SwerveModule m_backRight =
      new SwerveModule(DriveConstants.kBackRightDriveMotorPort,
                       DriveConstants.kBackRightSteerMotorPort,
                       DriveConstants.kBackRightCanCoderPort,
                       DriveConstants.kRearRightDriveEncoderReversed,
                       DriveConstants.kRearRightTurningEncoderReversed,
                       DriveConstants.kRearRightDriveMotorReversed,
                       DriveConstants.kRearRightTurningMotorReversed,
                       DriveConstants.kBackRightFaceForward);

  Field2d field2d;
  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  
  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry; 

  public void addVisionMeasurement(EstimatedRobotPose estimatedPose)
  {
    PhotonTrackedTarget closestTag = null;
    for ( final var tag : estimatedPose.targetsUsed )
    {
      if ( closestTag == null || tag.getBestCameraToTarget().getTranslation().getNorm() < closestTag.getBestCameraToTarget().getTranslation().getNorm() )
      {
        closestTag = tag;
      }
    }

    if ( closestTag == null )
    {
      return;
    }

    try
    {
      final var weights = getStddevForTag(estimatedPose, closestTag);
      m_odometry.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, weights);
    }
    catch (Exception e)
    {
      e.printStackTrace();
    }
  }

  public Optional<Pose2d> getNearestNode(){
    double allianceSign = (DriverStation.getAlliance() == Alliance.Blue) ? -1 : 1;
    double autoScoreTolerance = 2.0;
    var map = RVAprilTagField.pieceYMap;
    var botpose = getPose().getY();
    double resultPose;
    if (map.ceilingKey(botpose) == null){
        resultPose = map.lowerKey(botpose);
    } else if (map.lowerKey(botpose) == null){
        resultPose = map.ceilingKey(botpose);
    } else if (map.lowerKey(botpose) == null && map.ceilingKey(botpose) == null){
        return Optional.empty();
    } else {
        resultPose = Math.abs((botpose - map.ceilingKey(botpose))) <= Math.abs((botpose - map.lowerKey(botpose))) 
            ? map.ceilingKey(botpose) : map.lowerKey(botpose);
    }
    Pose2d computedPose = new Pose2d(
        new Translation2d((6.43)*allianceSign, resultPose), 
        new Rotation2d((Math.PI/2) - (Math.PI/2*allianceSign))
    );
    return (getPose().getTranslation().getDistance(computedPose.getTranslation()) >= autoScoreTolerance) ?
        Optional.empty() : Optional.of(computedPose);
  }

  Matrix<N3, N1> getStddevForTag(EstimatedRobotPose estimatedPose, PhotonTrackedTarget closestTarget)
  {
    final var distance = closestTarget.getBestCameraToTarget().getTranslation().getNorm();
    if ( distance < 3.0 || estimatedPose.targetsUsed.size() > 1 )
    {
      return VecBuilder.fill(0.1, 0.1, 10.0);
    }
    return VecBuilder.fill(distance / 10.0, distance / 10.0, Units.degreesToRadians(5.0 * (distance / 4.0)));
  } 

  public TimeInterpolatableBuffer<Rotation2d> gyroBuffer = TimeInterpolatableBuffer.createBuffer(1);

  public double initAngle;

  SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[]{
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_backLeft.getPosition(),
    m_backRight.getPosition()
  };

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem(RobotContainer robot) {
    this.field2d = robot.field;
    // m_odometry = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, 
    //   getAngle(), 
    //   swerveModulePositions,
    //   new Pose2d() 
    // );
    m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics, 
      getAngle(), 
      swerveModulePositions,
      new Pose2d(),
      VecBuilder.fill(0.01, 0.01, 0.01), //Drive standard deviations
      VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(10)) //Vision standard deviations
      );

    //Config PID Driving Motors
    m_frontLeft.configPIDdriving(0.0, 0.0, 0.0, Constants.ModuleConstants.kDriveMotorPID_F);
    m_frontRight.configPIDdriving(0.0, 0.0, 0.0, Constants.ModuleConstants.kDriveMotorPID_F);
    m_backLeft.configPIDdriving(0.0, 0.0, 0.0, Constants.ModuleConstants.kDriveMotorPID_F);
    m_backRight.configPIDdriving(0.0, 0.0, 0.0, Constants.ModuleConstants.kDriveMotorPID_F);
    //Config PID Turning Motors
    m_frontLeft.configPIDturning(0.9, 0.0, 0.09, 0.0);
    m_frontRight.configPIDturning(0.7, 0.0, 0.07, 0.0);
    m_backLeft.configPIDturning(0.8, 0.0, 0.0, 0.0);
    m_backRight.configPIDturning(0.7, 0.0, 0.06, 0.0);
  
    SmartDashboard.putData("field", field2d);

    if (Robot.isSimulation())
    {
      OdomSimulation.getInstance().attachOdom(m_odometry);
    }
  }

  public void enableRamping() {
    m_frontRight.configRamping(true);
    m_frontLeft.configRamping(true);
    m_backLeft.configRamping(true);
    m_backRight.configRamping(true);
  }

  public void disableRamping() {
    m_frontRight.configRamping(false);
    m_frontLeft.configRamping(false);
    m_backLeft.configRamping(false);
    m_backRight.configRamping(false);
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? 1.0 : -1.0));
  }

  private Pose2d currPos, pos;
  private Translation2d currXY;
  private Rotation2d currRot;
  private double currX, currY;

  int regulator = 0;
  @Override
  public void periodic() {
    gyroBuffer.addSample(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(getHeadingDegrees()));

    SmartDashboard.putNumber("RFAbs", m_frontRight.getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("LFAbs", m_frontLeft.getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("LRAbs", m_backLeft.getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("RRAbs", m_backRight.getAbsoluteEncoderPosition());
    // Update the odometry in the periodic block
    swerveModulePositions = new SwerveModulePosition[]{
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };

    m_frontLeft.dashboardLog("FL");
    m_frontRight.dashboardLog("FR");
    m_backLeft.dashboardLog("BL");
    m_backRight.dashboardLog("BR");

    m_odometry.update(
      new Rotation2d(Math.toRadians(getHeadingDegrees())),
      swerveModulePositions
    );

    currPos = getPose(); 
    currXY = currPos.getTranslation();
    currX = currXY.getX();
    currY = currXY.getY();
    currRot = currPos.getRotation();

  
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putNumber("PR Hypot", Math.hypot(getPitch(), getRoll()));

    SmartDashboard.putNumber("currX", currX);
    SmartDashboard.putNumber("currY", currY);
    SmartDashboard.putNumber("currHeadingDeg", currRot.getDegrees());

    field2d.setRobotPose(currPos.relativeTo(Constants.LEFT_BOTTOM_CORNER));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
   return m_odometry.getEstimatedPosition();
  }

  public void faceAllForward(boolean tryNotToMove) {
    m_frontRight.wheelFaceForward(DriveConstants.kFrontRightFaceForward, tryNotToMove);
    m_frontLeft.wheelFaceForward(DriveConstants.kFrontLeftFaceForward, tryNotToMove);
    m_backLeft.wheelFaceForward(DriveConstants.kBackLeftFaceForward, tryNotToMove);
    m_backRight.wheelFaceForward(DriveConstants.kBackRightFaceForward, tryNotToMove);
  }

  public boolean isAllCalibrated() {
    return m_frontRight.isCalibrated(DriveConstants.kFrontRightFaceForward) &&
           m_frontLeft.isCalibrated(DriveConstants.kFrontLeftFaceForward) &&
           m_backLeft.isCalibrated(DriveConstants.kBackLeftFaceForward) &&
           m_backRight.isCalibrated(DriveConstants.kBackRightFaceForward);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), swerveModulePositions, pose);
  }

  double maxLF, maxLR, maxRF, maxRR = 0;
  public static final double WHEEL_BASE_LENGTH = 30.0; //2022 prac; centimeters; 30.0
  public static final double WHEEL_BASE_WIDTH = 29.5; //2022 prac; centimeters; 29.5

  public void swerveDrive(double strafe, double forward, double omega, double driveSpeedScale) {
    // SmartDashboard.putNumber("driveStrafe", strafe);
    // SmartDashboard.putNumber("driveForward", forward);
    // SmartDashboard.putNumber("driveOmega", omega);

    //Drive speed scale is a reduction on the drive motor speed

    // If all of the joysticks are in the deadzone, don't update the motors
		// This makes side-to-side strafing much smoother
		if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
      stopDriveMotors(ControlMode.Velocity);
			return;
   	}
    double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
    double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);
    
    // Compute the constants used later for calculating speeds and angles
    double A = strafe - omegaL2;
    double B = strafe + omegaL2;
    double C = forward - omegaW2;
    double D = forward + omegaW2;
    
    // Compute the drive motor speeds
    double speedLF = speed(B, D)*driveSpeedScale;
    double speedLR = speed(A, D)*driveSpeedScale;
    double speedRF = speed(B, C)*driveSpeedScale;
    double speedRR = speed(A, C)*driveSpeedScale;
    
		// ... and angles for the steering motors 
		// When drives are calibrated for zero position on encoders they can be at 90 degrees
		// to the front of the robot. Subtract and add 90 degrees to steering calculation to offset
    // for initial position/calibration of drives if the drive zero position faces the side of
    // the robot.

		double angleLF = angle(B, D) ;
    double angleLR = angle(A, D) ;
    double angleRF = angle(B, C) ;
    double angleRR = angle(A, C) ;

    if(m_frontLeft.getDriveEncoderVelocity() > maxLF){
      maxLF = m_frontLeft.getDriveEncoderVelocity();
    }
    if(m_backLeft.getDriveEncoderVelocity() > maxLR){
      maxLR = m_backLeft.getDriveEncoderVelocity();
    }
    if(m_frontRight.getDriveEncoderVelocity() > maxRF){
      maxRF = m_frontRight.getDriveEncoderVelocity();
    }
    if(m_backRight.getDriveEncoderVelocity() > maxRR){
      maxRR = m_backRight.getDriveEncoderVelocity();
    }

    // Compute the maximum speed so that we can scale all the speeds to the range [0, 1]
    double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

    // Set each swerve module, scaling the drive speeds by the maximum speed
    setSwerveModule(m_frontLeft, angleLF, speedLF / maxSpeed, "LF");
    setSwerveModule(m_backLeft, angleLR, speedLR / maxSpeed, "BL");
    setSwerveModule(m_frontRight, angleRF, speedRF / maxSpeed, "RF");
		setSwerveModule(m_backRight, angleRR, speedRR / maxSpeed, "BR");
	}
	
  /**
   * @return The hypotenuse between (val1, val2)
   */
	private double speed(double val1, double val2){
    return Math.hypot(val1, val2);
  }
  
  /**
   * The angle between (val1, val2) on the coordinate plane and the positive X axis
   * @return The angle, in degrees, between -180 and 180
   */
  private double angle(double val1, double val2){
    return Math.toDegrees(Math.atan2(val1, val2));
  }

  private void setSwerveModule(SwerveModule module, double angle, double speed, String m_id) {
    double currentPosition = module.getTurningEncoderCount();
    double currentAngle = (currentPosition * 360 / Constants.ModuleConstants.kEncoderCPRSteer) % 360.0;
    
    if (currentAngle > 0.0) {
      if (currentAngle > 180.0) {
      currentAngle -= 360.0;
      }
    }
    else {
      if (currentAngle < -179.0) {
        currentAngle += 360.0;
      }
    }

    // This is because the steering encoders are inverted
    double targetAngle = -angle;
    double deltaDegrees = targetAngle - currentAngle;
    // If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }
    // If we need to turn more than 90 degrees, we can reverse the wheel direction instead and
		// only rotate by the complement
	
    if (Math.abs(deltaDegrees) > 90.0) {
      deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      speed = -speed;
    }
    
    double targetPosition;
    targetPosition = currentPosition + deltaDegrees * Constants.ModuleConstants.kEncoderCPRSteer / 360.0;

    SmartDashboard.putNumber(m_id+"TargetPostion", targetPosition);
    SmartDashboard.putNumber(m_id+"TargetSpeed", speed*Constants.ModuleConstants.kMaxDriveSpeedTicksPerSecond);
    SmartDashboard.putNumber(m_id+"CurrentPosition", currentPosition);
    SmartDashboard.putNumber(m_id+"CurrentAngle", currentAngle);

    module.setDriving(ControlMode.Velocity, speed*Constants.ModuleConstants.kMaxDriveSpeedTicksPerSecond); //previous value was 600 for Talon SRX
    module.setTurning(ControlMode.Position, targetPosition);
	}

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    //SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setSwerveModule(m_frontLeft, desiredStates[0].angle.getDegrees(), desiredStates[0].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond, "LF");
    setSwerveModule(m_frontRight, desiredStates[1].angle.getDegrees(), desiredStates[1].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond, "RF");
    setSwerveModule(m_backLeft, desiredStates[2].angle.getDegrees(), desiredStates[2].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond, "BL");
    setSwerveModule(m_backRight, desiredStates[3].angle.getDegrees(), desiredStates[3].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond, "BR");
  }


  public void stopDriveMotors(ControlMode mode) {
    m_frontLeft.setDriving(mode, 0.0);
    m_backLeft.setDriving(mode, 0.0);
    m_frontRight.setDriving(mode, 0.0);
    m_backRight.setDriving(mode, 0.0);
  }

  public void setTurnMotors(ControlMode mode, double val) {
    m_frontLeft.setTurning(mode, val);
    m_backLeft.setTurning(mode, val);
    m_frontRight.setTurning(mode, val);
    m_backRight.setTurning(mode, val);
  }

  public void zeroHeading() {
    m_gyro.reset();
    initAngle = 0;
  }

  /**
   * Returns the heading of the robot relative to the field
   * @return The angle in degrees
   */
  public double getHeadingDegrees() {
    return (m_gyro.getAngle()-initAngle) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getnavxFusedHeading(){
    return m_gyro.getFusedHeading();
  }

  /**
   * @return The angle, forward and back, in degrees. Forward is positive
   */
  public double getPitch() {
    return m_gyro.getPitch();
  }

  /**
   * @return The angle, left and right, in degrees. Leftward is positive
   */
  public double getRoll() {
    return m_gyro.getRoll();
  }

  public double getSpeedX(){
    return m_gyro.getVelocityX();
  }

  public double getAccX(){
    return m_gyro.getVelocityY();
  }
}