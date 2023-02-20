// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

// Aaron was here :D

public class ArmElevatorSubsystem extends SubsystemBase {

  DoubleSolenoid elevatorLDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.kLforward, PneumaticConstants.kLreverse);
  DoubleSolenoid elevatorRDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.kRforward, PneumaticConstants.kRreverse);

  /** Creates a new ExampleSubsystem. */
  public ArmElevatorSubsystem() {}

  public void Extend() {
    elevatorLDoublePCM.set(kForward);
    elevatorRDoublePCM.set(kForward);
  }
  public void Retract() {
    elevatorLDoublePCM.set(kReverse);
    elevatorRDoublePCM.set(kReverse);
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
