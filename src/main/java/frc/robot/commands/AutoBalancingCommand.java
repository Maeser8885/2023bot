// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoBalancingCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private int state; //0 = driving, 1 = adjusting backwards, 2 = adjusting forwards
  private int ticks;
  private boolean balanced;
  private ADIS16470_IMU gyro ;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoBalancingCommand(DriveSubsystem subsystem, ADIS16470_IMU imu) {
    m_subsystem = subsystem;
    gyro = imu;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    ticks = 0;
    balanced = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (balanced) {
      return;
    }
    if (state == 0) {
      if (gyro.getXComplementaryAngle() < 0) {
        state = 1;
        ticks = 0;
      } else {
        m_subsystem.driveArcade( .5, 0);
      }
    } else if (state == 1) {
      if (gyro.getXComplementaryAngle() > 0) {
        if (ticks < Constants.autoConst) {
          balanced = true;
          return;
        }
        ticks = 0;
        state = 2;
      } else {
        m_subsystem.driveArcade( -.5, 0);
        ticks += 1;
      }
    } else if (state == 2) {
      ticks = 0;
      if (gyro.getXComplementaryAngle() < 0) {
        if (ticks < Constants.autoConst) {
          balanced = true;
          return;
        }
        ticks = 0;
        state = 1;
      } else {
        m_subsystem.driveArcade( .5, 0);
        ticks += 1;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return balanced;
  }
}
