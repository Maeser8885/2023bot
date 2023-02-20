// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalancingCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoDropoffCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final GripperSubsystem m_gripper = new GripperSubsystem();
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();// A simple auto routine that drives forward a specified distance, and then stops.


  // ADIS Gyro
  public ADIS16470_IMU gyro = new ADIS16470_IMU();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_secondaryDriverController =
      new CommandXboxController(OperatorConstants.kSecondaryDriverControllerPort);
  private final CommandJoystick m_driverJoystick =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  //MAKE COMMANDS HERE!!
  // Once on the Charging Station, it balances hopefully.
  private final Command m_balanceAuto = new AutoBalancingCommand(m_driveSubsystem, gyro);
  // Literally just waits there, menacingly.
  private final Command m_waitAuto = new WaitCommand(3);
  // Moves the robot a set ammount.
  private final Command m_moveAuto = new AutoDriveCommand(m_driveSubsystem);
  // Moves the robot to Scoring Area, drops the game piece.
  private final Command m_dropoffAuto = new AutoDropoffCommand(m_gripper);
  // placeholder command
  private final Command m_complexAuto = new RunCommand(()->{});

  
  SendableChooser<Command> m_autochooser0 = new SendableChooser<>();
  SendableChooser<Command> m_autochooser1 = new SendableChooser<>();
  SendableChooser<Command> m_autochooser2 = new SendableChooser<>();
  SendableChooser<Command> m_autochooser3 = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveSubsystem.setDefaultCommand(new RunCommand(()->{
      m_driveSubsystem.driveArcade(-m_driverJoystick.getY() *adjustThrottle(-m_driverJoystick.getThrottle()), -m_driverJoystick.getTwist()*adjustThrottle(-m_driverJoystick.getThrottle()));
    }, //TODO: Rotation throttle?
    m_driveSubsystem));
    /*m_driveSubsystem.setDefaultCommand(new RunCommand(()->{
      m_driveSubsystem.driveArcade(-m_driverJoystick.getY(), -m_driverJoystick.getTwist());
    }, //TODO: Rotation throttle?
    m_driveSubsystem));*/

    // Aaron was here :D

    // A chooser for autonomous commands

    // Add commands to the autonomous command chooser
    m_autochooser0.addOption("Wait Auto", m_waitAuto);
    m_autochooser0.addOption("Dropoff Auto", m_dropoffAuto); 
    m_autochooser0.addOption("Move Auto", m_moveAuto);
    m_autochooser0.addOption("Balance Auto", m_balanceAuto);

    m_autochooser1.addOption("Dropoff Auto", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser1.addOption("Move Auto", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser1.addOption("", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser1.addOption("", m_complexAuto); //placeholder for ACTUAL COMMAND

    m_autochooser2.addOption("Dropoff Auto", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser2.addOption("Move Auto", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser2.addOption("", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser2.addOption("", m_complexAuto); //placeholder for ACTUAL COMMAND

    m_autochooser3.addOption("Dropoff Auto", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser3.addOption("Move Auto", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser3.addOption("", m_complexAuto); //placeholder for ACTUAL COMMAND
    m_autochooser3.addOption("", m_complexAuto); //placeholder for ACTUAL COMMAND

    // Put the chooser on the dashboard
    SmartDashboard.putData("test0", m_autochooser0);
    SmartDashboard.putData("test1", m_autochooser1);
    SmartDashboard.putData("test2", m_autochooser2);
    SmartDashboard.putData("test3", m_autochooser3);

  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //m_driverJoystick.(new RunCommand(()->{
      //m_driveSubsystem.driveArcade(m_driverJoystick.getLeftY(), m_driverJoystick.getLeftX());
    //}, m_driveSubsystem));

     //set up gripper open/close (WILL NOT RUN UNLESS SWITCHED OUT FOR JOYSTICK)
    //  new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
    //  .onTrue(new InstantCommand(() -> m_gripper.openGripper()))
    //  .onFalse(new InstantCommand(() -> m_gripper.closeGripper()));
  }

  private double adjustThrottle(double throttle) {
    return throttle/2 +.5;
  }

  public double getGyroReading() {
    gyro.getAngle();
    gyro.getXComplementaryAngle();
    gyro.getYComplementaryAngle();
    return 0.0;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(this.m_autochooser0.getSelected(), this.m_autochooser1.getSelected(), this.m_autochooser2.getSelected(), this.m_autochooser3.getSelected());
  }
}
