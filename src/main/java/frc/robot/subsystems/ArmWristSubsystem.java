// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

public class ArmWristSubsystem extends SubsystemBase {

  private CANSparkMax m_WRISTneo = new CANSparkMax(MotorConstants.kWRISTSparkMax, MotorType.kBrushless);
  private RelativeEncoder m_encoder;

  private double m_setpoint;
  private double m_prevSetpoint;
  private SparkMaxPIDController m_controller;


  private TrapezoidProfile m_profile;
  private Timer m_timer;
  
  private TrapezoidProfile.State targetState;
  private double feedforward;
  private double manualValue;

  /** Creates a new ExampleSubsystem. */
  public ArmWristSubsystem() {

    m_encoder = m_WRISTneo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_encoder.setPositionConversionFactor(Constants.ArmWrist.kPositionFactor);
    m_encoder.setVelocityConversionFactor(Constants.ArmWrist.kVelocityFactor);

    m_setpoint = Constants.ArmPivot.kHomePosition;

    m_controller = m_WRISTneo.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.ArmWrist.kPositionPIDGains);

    m_setpoint = Constants.ArmWrist.kHomePosition;

    m_timer = new Timer();
    m_timer.start();
    m_timer.reset();

  }

  public void setTargetPosition(double _setpoint, GripperSubsystem _gripper) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.ArmPivot.kArmMotionConstraint, goal, state);
    m_timer.reset();
  }
  public void runManual(double _power) {
    //reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
    m_setpoint = m_encoder.getPosition();
    targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.ArmPivot.kArmMotionConstraint, targetState, targetState);
    //update the feedforward variable with the newly zero target velocity
    feedforward = Constants.ArmPivot.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.ArmPivot.kArmZeroCosineOffset, targetState.velocity);
    m_WRISTneo.set(_power + (feedforward / 12.0));
    manualValue = _power;
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
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
