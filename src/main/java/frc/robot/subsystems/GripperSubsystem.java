// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
  
  Solenoid gripSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private boolean gripperState = false; // opened = false, closed = true

  /** Creates a new ExampleSubsystem. */
  public GripperSubsystem() {
  }
   
  public void openGripper() {
    gripSolenoidPCM.set(false);
    gripperState = false;
  }
  public void closeGripper() {
    gripSolenoidPCM.set(true);
    gripperState = true;
  } 
  public void toggleGripper() {
    gripperState = !gripperState;
    gripSolenoidPCM.set(gripperState);
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
    SmartDashboard.putBoolean("Gripper Open State" , !gripperState);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
