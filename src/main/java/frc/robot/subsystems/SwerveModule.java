
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final TalonFX m_driveMotor, m_turningMotor;
  private final CANCoder m_absoluteEncoder;
  int turningMotorChannel;

  private int m_faceForwardOffset = 0;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  //Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController
      = new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController, 0, 0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param encoderChannel
   */
  public SwerveModule(int driveMotorChannel,
                      int turningMotorChannel,
                      int encoderChannel,
                      boolean driveEncoderReversed,
                      boolean turningEncoderReversed,
                      boolean driveMotorReversed,
                      boolean turningMotorReversed,
                      int faceForwardOffset) {

    m_faceForwardOffset = faceForwardOffset;
    this.turningMotorChannel = turningMotorChannel;

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_absoluteEncoder = new CANCoder(encoderChannel);
    
    m_driveMotor.configFactoryDefault();
    m_turningMotor.configFactoryDefault();

    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    
    //Set whether drive encoder should be reversed or not
    m_driveMotor.setSensorPhase(driveEncoderReversed); //<<There is a similar function in talon class
    m_driveMotor.setInverted(driveMotorReversed); // Revert the motor direction
    m_turningMotor.setInverted(turningMotorReversed);

    //Set whether turning encoder should be reversed or not
    m_turningMotor.setSensorPhase(turningEncoderReversed); 

    m_driveMotor.setSelectedSensorPosition(0,0,10);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Coast);

    configRamping(false);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    wheelFaceForward(faceForwardOffset,false);
    //
    m_driveMotor.setSelectedSensorPosition(0);// This should reduce the size of the numbers on the encoder
  }

  public void configRamping(boolean enabled) {
    m_driveMotor.configOpenloopRamp(enabled ? 0.25 : 0, 10);
    m_driveMotor.configClosedloopRamp(enabled ? 0.25 : 0, 10);
  }

  public boolean isCalibrated(double faceForwardOffset)
  {
    double currangle = m_absoluteEncoder.getAbsolutePosition();
    double theta = (360 - (currangle - faceForwardOffset)) % 360;
    double thetaTicks = (theta/360)*Constants.ModuleConstants.kEncoderCPRSteer;
    double setTicks = -m_turningMotor.getSelectedSensorPosition();
    boolean calib = Math.abs(thetaTicks - setTicks) < 50;
    SmartDashboard.putBoolean("calib"+turningMotorChannel, calib);
    SmartDashboard.putNumber("calib"+turningMotorChannel+"error", thetaTicks - setTicks);

    return calib;
  }
  
  public void wheelFaceForward(double faceForwardOffset, boolean tryNotToMove) {
    double currangle = m_absoluteEncoder.getAbsolutePosition();
    double theta = (360 - (currangle - faceForwardOffset)) % 360;
    double thetaTicks = (theta/360)*Constants.ModuleConstants.kEncoderCPRSteer;
    SmartDashboard.putNumber("SwerveInitTicks"+turningMotorChannel, thetaTicks);

    ErrorCode err = m_turningMotor.setSelectedSensorPosition(-thetaTicks);
    int count = 0;
    while ( err != ErrorCode.OK )
    {
      System.out.println("Failed to zero "+turningMotorChannel+": "+err);
      err = m_turningMotor.setSelectedSensorPosition(-thetaTicks);
      if ( count > 5) 
      {
        break;
      }
      count++;
    }
    if (tryNotToMove)
    {
      m_turningMotor.set(ControlMode.Position, -thetaTicks);
    }
    else
    {
      m_turningMotor.set(ControlMode.Position, 0);
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    //converting ticks per 100 miliseconds to meters per second
    var heading = new Rotation2d(m_turningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderDistancePerPulse);
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity()*10*ModuleConstants.kDriveEncoderDistancePerPulse, heading);
  }


  public SwerveModulePosition getPosition() {
    var distance = m_driveMotor.getSelectedSensorPosition()/ModuleConstants.kDriveTicksPerMeter;
    var rotation = new Rotation2d(m_turningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderDistancePerPulse);
    return new SwerveModulePosition(distance, rotation);
  }

  public void dashboardLog(String name)
  {
    var pos = getPosition();
    SmartDashboard.putNumber(name+"heading", pos.angle.getDegrees());
    SmartDashboard.putNumber(name+"pos", pos.distanceMeters);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    // final var driveOutput = m_drivePIDController.calculate(
    //     m_driveMotor.getSelectedSensorVelocity(0)*10*ModuleConstants.kDriveEncoderDistancePerPulse,
    //                   state.speedMetersPerSecond);
    // Calculate the turning motor output from the turning PID controller.
    // final var turnOutput = m_turningPIDController.calculate(
    //     m_turningMotor.getSelectedSensorPosition(0)*ModuleConstants.kTurningEncoderDistancePerPulse, state.angle.getRadians()
    // );

    //SwerveModuleState state =
    //      optimize(desiredState, new Rotation2d(m_turningMotor.getSelectedSensorPosition()/Constants.ModuleConstants.kTurningEncoderDistancePerPulse));

    desiredAngle = state.angle.getRadians()/Constants.ModuleConstants.kTurningEncoderDistancePerPulse;//in ticks
    currentAngle = m_turningMotor.getSelectedSensorPosition();//in ticks
    desiredDriveSpeed = state.speedMetersPerSecond/ Constants.ModuleConstants.kDriveEncoderDistancePerPulse / 10;

    optimizedAngle();
    System.out.println("Got to swerve module: Angle, Speed: " + desiredAngle + desiredDriveSpeed);
    m_turningMotor.set(ControlMode.Position, desiredAngle);
    m_driveMotor.set(ControlMode.Velocity, desiredDriveSpeed);

    //Troubleshooting Statements  
    //m_driveMotor.set(ControlMode.Velocity, 100.0);
    //m_turningMotor.set(ControlMode.Position, 0);//
    //m_turningMotor.set(ControlMode.PercentOutput, .1);
  }

  private boolean desiredAngleSign, currentAngleSign;
  private double desiredAngle, currentAngle = 0.0;
  private double desiredDriveSpeed;
  private int currentAngleInt;

  public void optimizedAngle(){
    if(desiredAngle>0){desiredAngleSign = true;} 
    else {desiredAngleSign = false;}
    if(currentAngle>0){ currentAngleSign = true;} 
    else {currentAngleSign = false;}

    currentAngleInt = (int) currentAngle;

    if(desiredAngleSign != currentAngleSign && Math.abs(desiredAngle-currentAngle) > 900 && currentAngleInt > 0){
      m_turningMotor.setSelectedSensorPosition(m_turningMotor.getSelectedSensorPosition()-Constants.ModuleConstants.kEncoderCPRSteer);
    } else if(desiredAngleSign != currentAngleSign && Math.abs(desiredAngle-currentAngle) > 900 && currentAngleInt < 0){
      m_turningMotor.setSelectedSensorPosition(m_turningMotor.getSelectedSensorPosition()+Constants.ModuleConstants.kEncoderCPRSteer);
    }
    // if(Math.abs(desiredAngle-currentAngle)>256 && Math.abs(desiredAngle-currentAngle)<800 && desiredAngle > 0){
    //   desiredDriveSpeed = -desiredDriveSpeed;
    //   desiredAngle = desiredAngle - 512 ;
    // } else if(Math.abs(desiredAngle-currentAngle)>256 && Math.abs(desiredAngle-currentAngle)<800 && desiredAngle < 0){
    //   desiredDriveSpeed = -desiredDriveSpeed;
    //   desiredAngle = 512 + desiredAngle;
    // }
    SmartDashboard.putNumber("Difference", desiredAngle-currentAngle);
  }

  public void setDesiredState2(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        optimize(desiredState, new Rotation2d(m_turningMotor.getSelectedSensorPosition()*(360/Constants.ModuleConstants.kEncoderCPRSteer))); //In degrees

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate ((m_driveMotor.getSelectedSensorVelocity()/10)*Constants.ModuleConstants.kDriveEncoderDistancePerPulse, state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition()*Constants.ModuleConstants.kTurningEncoderDistancePerPulse, state.angle.getRadians());

    SmartDashboard.putNumber("DriveOutput", driveOutput);
    SmartDashboard.putNumber("TurnOutput", turnOutput);

    // Calculate the turning motor output from the turning PID controller.
    //m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
    //m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
  }

  public static SwerveModuleState optimize(
    SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0 && Math.abs(delta.getDegrees()) < 340){
      return new SwerveModuleState(
         -desiredState.speedMetersPerSecond,
         desiredState.angle.rotateBy(Rotation2d.fromDegrees(180)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
    //return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
  }

  //Swerve Module Motor Methods VV

  public void setTurning(ControlMode mode, double speed){
    m_turningMotor.set(mode, speed);
  }

  public void setDriving(ControlMode mode, double speed){
    m_driveMotor.set(mode, speed);
  }

  public void configPIDturning(double P, double I, double D, double F){
    m_turningMotor.config_kP(0,P, 0);
    m_turningMotor.config_kI(0,I, 0);
    m_turningMotor.config_kD(0,D, 0);
    m_turningMotor.config_kF(0,F, 0);
    m_driveMotor.configAllowableClosedloopError(0, 5, 0);
  }

  public void configPIDdriving(double P, double I, double D, double F){
    m_driveMotor.config_kP(0,P, 0);
    m_driveMotor.config_kI(0,I, 0);
    m_driveMotor.config_kD(0,D, 0);
    m_driveMotor.config_kF(0,F, 0);
  }
  //Swerve Module Encoder Methods VV

  public void zeroSteerHeading(){
    m_turningMotor.set(ControlMode.Position,0);
  }

  public void setDriveMotor(double output) {
    m_driveMotor.set(ControlMode.Velocity, output);
  }

  public void setSteerMotor(ControlMode mode, double output) {
    m_turningMotor.set(mode, output);
  }

  public void zeroSteerEncoder(){
    m_turningMotor.setSelectedSensorPosition(0);
  }

  public void setSteerEncoder(double sensorPos){
    m_turningMotor.setSelectedSensorPosition(sensorPos);
  }

  public void zeroDriveEncoder(){
    m_driveMotor.setSelectedSensorPosition(0);
  }

  public void resetEncoders()
  {
    m_driveMotor.setSelectedSensorPosition(0,0,10);        
    m_turningMotor.setSelectedSensorPosition(0,0,10);
  }

  public double getDriveEncoderCount(){
    return m_driveMotor.getSelectedSensorPosition();
  }

  public double getDriveEncoderVelocity(){
    return m_driveMotor.getSelectedSensorVelocity(0);
  }

  public double getTurningEncoderCount(){
    return m_turningMotor.getSelectedSensorPosition();
  }

  public double getAbsoluteEncoderPosition(){
    //System.out.println("CanCoderPos " + m_absoluteEncoder.getAbsolutePosition()+"; TalonEncoder "+m_turningMotor.getSelectedSensorPosition());
    return m_absoluteEncoder.getAbsolutePosition();
  }
  
  public int getAbsoluteEncoderPositionTicks() {
    double num = m_absoluteEncoder.getAbsolutePosition() /360.0 * 4096.0;
    return (int) num;//Constants.ModuleConstants.kAbsoluteEncoderCPRSteer;
  }
}