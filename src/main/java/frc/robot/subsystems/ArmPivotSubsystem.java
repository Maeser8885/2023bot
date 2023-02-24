// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
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

// Aaron was here :D
// https://github.com/REVrobotics/2023-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/subsystems/GripperSubsystem.java

public class ArmPivotSubsystem extends SubsystemBase {
  private RelativeEncoder m_encoder;

  private CANSparkMax m_LPIVOTneo = new CANSparkMax(MotorConstants.kLARMSparkMax, MotorType.kBrushless);
  private CANSparkMax m_RPIVOTneo = new CANSparkMax(MotorConstants.kRARMSparkMax, MotorType.kBrushless);

  private double m_setpoint;
  private double m_prevSetpoint;
  private SparkMaxPIDController m_controller;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  
  private TrapezoidProfile.State targetState;
  private double feedforward;
  private double manualValue;

  /** Creates a new ExampleSubsystem. */
  public ArmPivotSubsystem() {

    m_RPIVOTneo.follow(m_LPIVOTneo, true);
  

    m_encoder = m_LPIVOTneo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_encoder.setPositionConversionFactor(Constants.ArmPivot.kPositionFactor);
    m_encoder.setVelocityConversionFactor(Constants.ArmPivot.kVelocityFactor);

    m_setpoint = Constants.ArmPivot.kHomePosition;

    m_controller = m_LPIVOTneo.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.ArmPivot.kPositionPIDGains);
 
    m_setpoint = Constants.ArmPivot.kHomePosition;

    m_timer = new Timer();
    m_timer.start();
    m_timer.reset();

    updateMotionProfile();
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
    m_LPIVOTneo.set(_power + (feedforward / 12.0));
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

  public void testMethod() {
    
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
