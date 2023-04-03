// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PneumaticConstants;

// Aaron was here :D

public class ArmElevatorSubsystem extends SubsystemBase {
  public Compressor armCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  DoubleSolenoid elevatorLDouble = new DoubleSolenoid(Constants.PneumaticConstants.kSecondaryPCM, PneumaticsModuleType.CTREPCM, PneumaticConstants.kL_PortForward, PneumaticConstants.kL_PortReverse);
  DoubleSolenoid elevatorRDouble = new DoubleSolenoid(Constants.PneumaticConstants.kSecondaryPCM, PneumaticsModuleType.CTREPCM, PneumaticConstants.kR_PortForward, PneumaticConstants.kR_PortReverse);

  /** Creates a new ExampleSubsystem. */
  public ArmElevatorSubsystem() {
    Retract();
  }

  public void Extend() {
    elevatorLDouble.set(kForward);
    elevatorRDouble.set(kForward);
  }
  public boolean getExtended(){
    return elevatorLDouble.get() == kForward;
  }
  public void Retract() {
    elevatorLDouble.set(kReverse);
    elevatorRDouble.set(kReverse);
  }

  public void toggle(){
    elevatorLDouble.toggle();
    elevatorRDouble.toggle();
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
