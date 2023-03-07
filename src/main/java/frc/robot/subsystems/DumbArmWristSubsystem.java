// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

public class DumbArmWristSubsystem extends SubsystemBase {

  private final SparkMaxPIDController m_controller;
  private CANSparkMax m_WRISTneo = new CANSparkMax(MotorConstants.kWRISTSparkMax, MotorType.kBrushless);
  private RelativeEncoder m_encoder;

  private double m_setpoint;
  private double m_prevSetpoint;
  //private SparkMaxPIDController m_controller;

  /** Creates a new ExampleSubsystem. */
  public DumbArmWristSubsystem() {

    m_encoder = m_WRISTneo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    m_controller = m_WRISTneo.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.ArmWrist.kPositionPIDGains);
    m_WRISTneo.setInverted(false);

    m_setpoint = Constants.ArmWrist.kWHomePosition;

  }

  public void setPosition(double position){
    m_setpoint = position;
  }

  public void changePosition(double amount){
    // TODO TEST THIS TO GET BOUNDS
    /*if (m_setpoint + amount >= Constants.ArmWrist.kMaxPosition){
      m_setpoint = Constants.ArmWrist.kMaxPosition;
    } else if (m_setpoint + amount <= Constants.ArmWrist.kWHomePosition){
      m_setpoint = Constants.ArmWrist.kWHomePosition;
    }
    else {
      m_setpoint += amount;
    }*/
    m_setpoint += amount;
  }

  // Positive is out, Negative is in.

  /*public void set(double speed){
    m_WRISTneo.set(speed);
  }*/





  @Override
  public void periodic() { // This method will be called once per scheduler run
    if (m_setpoint != m_prevSetpoint) {
      m_controller.setReference(m_setpoint, CANSparkMax.ControlType.kPosition);
    }
    m_prevSetpoint = m_setpoint;
    SmartDashboard.putNumber("Wrist Angle", m_encoder.getPosition());
    SmartDashboard.putNumber("Expected Angle", m_setpoint);
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
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
