// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * WPILib's DutyCycleEncoderSim is missing setAbsolutePosition()
 */

package frc.robot.subsystems.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** Class to control a simulated duty cycle encoder. */
public class FixedDutyCycleEncoderSim {
  private final SimDouble m_simPosition;
  private final SimDouble m_simAbsolutePosition;
  private final SimDouble m_simDistancePerRotation;

  /**
   * Constructs from an DutyCycleEncoder object.
   *
   * @param encoder DutyCycleEncoder to simulate
   */
  public FixedDutyCycleEncoderSim(DutyCycleEncoder encoder) {
    this(encoder.getSourceChannel());
  }

  /**
   * Constructs from a digital input channel.
   *
   * @param channel digital input channel.
   */
  public FixedDutyCycleEncoderSim(int channel) {
    SimDeviceSim wrappedSimDevice = new SimDeviceSim("DutyCycle:DutyCycleEncoder", channel);
    m_simPosition = wrappedSimDevice.getDouble("position");
    m_simAbsolutePosition = wrappedSimDevice.getDouble("absPosition");
    m_simDistancePerRotation = wrappedSimDevice.getDouble("distance_per_rot");
  }

  /**
   * Set the position in turns.
   *
   * @param turns The position.
   */
  public void set(double turns) {
    m_simPosition.set(turns);
  }

  /**
   * Set the position.
   *
   * @param distance The position.
   */
  public void setDistance(double distance) {
    m_simPosition.set(distance / m_simDistancePerRotation.get());
  }


  public void setAbsolutePosition(double position)
  {
    m_simAbsolutePosition.set(position);
  }
}
