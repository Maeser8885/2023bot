// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

// Aaron was here :D

public class DriveSubsystem extends SubsystemBase {
  // Declares physical parts
  private CANSparkMax FLneo = new CANSparkMax(MotorConstants.kFLSparkMax, MotorType.kBrushless);
  private CANSparkMax FRneo = new CANSparkMax(MotorConstants.kFRSparkMax, MotorType.kBrushless);
  private CANSparkMax BLneo = new CANSparkMax(MotorConstants.kBLSparkMax, MotorType.kBrushless);
  private CANSparkMax BRneo = new CANSparkMax(MotorConstants.kBRSparkMax, MotorType.kBrushless);
  private MotorControllerGroup LeftMotorGroup = new MotorControllerGroup(FLneo, BLneo);
  private MotorControllerGroup RightMotorGroup = new MotorControllerGroup(FRneo, BRneo);
  private DifferentialDrive differentialDrive;

  private RelativeEncoder FLEncoder;
  private RelativeEncoder FREncoder;
  private RelativeEncoder BLEncoder;
  private RelativeEncoder BREncoder;

  private RelativeEncoder[] m_encoders = {FLEncoder,FREncoder,BLEncoder,BREncoder};
  /*private Spark LMotors = new Spark(1);
  private Spark RMotors = new Spark(0);*/
  //private DifferentialDrive differentialDrive = new DifferentialDrive(FLneo, FRneo);



  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {

    RightMotorGroup.setInverted(true);
    //BRneo.setInverted(true);
    differentialDrive = new DifferentialDrive(LeftMotorGroup, RightMotorGroup);
    //RMotors.setInverted(true);
  }

  public void driveArcade(double xspeed, double zrot) {
    differentialDrive.arcadeDrive(xspeed, zrot);
  }
  public void driveCurvature(double xspeed, double zrot, boolean turnInPlace) {
    differentialDrive.curvatureDrive(xspeed, zrot, turnInPlace);
  }
  public void resetEncoders(){
    for (RelativeEncoder encoder: m_encoders){
      encoder.setPosition(0); //TODO FINISH THIS
    }
  }

  public double getAverageEncoders(){
    return FLEncoder.getPosition() + FREncoder.getPosition() + BLEncoder.getPosition() + BREncoder.getPosition() / 4;
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
    SmartDashboard.putNumber("FL", FLneo.get());
    SmartDashboard.putNumber("FR", FRneo.get());
    SmartDashboard.putNumber("BL", BLneo.get());
    SmartDashboard.putNumber("BR", BRneo.get());
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
