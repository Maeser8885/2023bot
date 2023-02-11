// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxRelativeEncoder;

import frc.lib.PIDGains;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

public class GripperSubsystem extends SubsystemBase {
  private CANSparkMax GRIPneo = new CANSparkMax(MotorConstants.kGRIPSparkMax, MotorType.kBrushless);
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_controller;
  private double m_setpoint;
  private double m_prevSetpoint;

  //Thanks to the people coding the 2023 REV ION Robotics Starter Bot for sharing their code!
  /** Creates a new ExampleSubsystem. */
  public GripperSubsystem() {
    m_encoder = GRIPneo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    m_controller = GRIPneo.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Gripper.kPositionPIDGains);

    m_setpoint = Constants.Gripper.kClosePosition;
  }

  public boolean isSafe() {
    return m_encoder.getPosition() > Constants.Gripper.kSafePosition;
  }
  public void openGripper() {
    m_setpoint = Constants.Gripper.kOpenPosition;
  }
  public void closeGripper() {
    m_setpoint = Constants.Gripper.kClosePosition;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    if (m_setpoint != m_prevSetpoint) {
      m_controller.setReference(m_setpoint, CANSparkMax.ControlType.kPosition);
    }
    m_prevSetpoint = m_setpoint;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
