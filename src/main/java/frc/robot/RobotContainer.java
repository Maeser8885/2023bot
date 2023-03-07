// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmPivot;
import frc.robot.commands.Commands;
import frc.robot.commands.DriveTimedCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer instance;

  public static synchronized RobotContainer getInstance(){
    if (instance == null){
      instance = new RobotContainer();
    }
    return instance;
  }

  // The robot's subsystems and commands are defined here...
  public final GripperSubsystem m_gripper = new GripperSubsystem();
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final ArmPivotSubsystem m_armPivot=null;// = new ArmPivotSubsystem();
  public final DumbArmPivotSubsystem m_dumbPivot = new DumbArmPivotSubsystem();

  public final DumbArmWristSubsystem m_dumbWrist = new DumbArmWristSubsystem();
  public final ArmElevatorSubsystem m_elevator = new ArmElevatorSubsystem();

  // ADIS Gyro
  public ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  public Controls m_controls = Controls.getInstance();


  //private final Solenoid fake;
  
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  //SendableChooser<Command> m_autochooser1 = new SendableChooser<>();
  //SendableChooser<Command> m_autochooser2 = new SendableChooser<>();
  //SendableChooser<Command> m_autochooser3 = new SendableChooser<>();

  SendableChooser<RunCommand> m_drivechooser = new SendableChooser<>();

  private final RunCommand defaultDriveCommand = new RunCommand(()->{
    m_driveSubsystem.driveArcade(-m_controls.getThrottledY(), -m_controls.getThrottledTwist());
  }, m_driveSubsystem);

  private final RunCommand richardDriveCommand = new RunCommand(()->{
    m_driveSubsystem.driveArcade(-m_controls.getThrottledY(), m_controls.getThrottledX());
  }, m_driveSubsystem);

  private final RunCommand curvatureDriveCommand = new RunCommand(()->{
    m_driveSubsystem.driveCurvature(-m_controls.getThrottledY(), m_controls.getThrottledTwist(), m_controls.m_driverJoystick.button(11).getAsBoolean());
  }, m_driveSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //fake = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    // Configure the trigger bindings
    configureBindings();
    setupDashboard();

    defaultDriveCommand.setName("defaultDrive");
    richardDriveCommand.setName("richard");
    curvatureDriveCommand.setName("curvature");

    m_driveSubsystem.setDefaultCommand(defaultDriveCommand);
    m_dumbPivot.setDefaultCommand(new RunCommand(() -> m_dumbPivot.changePos(m_controls.deadband(m_controls.m_secondaryDriverController.getLeftY())*-0.5),m_dumbPivot));
    m_dumbWrist.setDefaultCommand(new RunCommand(() -> m_dumbWrist.changePosition(m_controls.deadband(m_controls.m_secondaryDriverController.getRightY())*0.5),m_dumbWrist));
    /*m_driveSubsystem.setDefaultCommand(new RunCommand(()->{
      m_driveSubsystem.driveArcade(-m_driverJoystick.getY(), -m_driverJoystick.getTwist());
    }, //TODO: Rotation throttle?
    m_driveSubsystem));*/

    // Aaron was here :D
    CameraServer.startAutomaticCapture();
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

    // Button Configuring!!
    m_controls.switchDriveButton.onTrue(new InstantCommand(()->{
      CommandScheduler.getInstance().cancel(m_driveSubsystem.getDefaultCommand());
      this.m_driveSubsystem.setDefaultCommand(this.m_drivechooser.getSelected());
      System.out.println(this.m_drivechooser.getSelected().getName());
    }));

    System.out.println("Configured Button Bindings");
    m_controls.gripperButton.onTrue(new InstantCommand(m_gripper::toggleGripper,m_gripper));
    m_controls.extendButton.onTrue(new InstantCommand(() -> {
      if(m_dumbPivot.m_encoder.getPosition() < ArmPivot.kSafeExtendLimit){
        m_elevator.toggle();
      } else {
        System.out.println("ARM IS TOO LOW!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      }

    },m_elevator));
    /*if (m_gripper != null) {
      m_controls.gripperButton.onTrue(new InstantCommand(m_gripper::toggleGripper));
    }
    if (m_armPivot != null){
      //m_controls.highScoringButton.onTrue(new InstantCommand(() -> {m_armPivot.setTargetPosition(ArmPivot.kHighScoringPosition);}));
      //m_controls.lowScoringButton.onTrue(new InstantCommand(() -> {m_armPivot.setTargetPosition(ArmPivot.kLowScoringPosition);}));
      //m_controls.highIntakeButton.onTrue(new InstantCommand(() -> {m_armPivot.setTargetPosition(ArmPivot.kHighIntakePosition);}));
      //m_controls.IntakeButton.onTrue(new InstantCommand(() -> {m_armPivot.setTargetPosition(ArmPivot.kIntakePosition);}));
      //m_controls.HomeButton.onTrue(new InstantCommand(() -> {m_armPivot.setTargetPosition(ArmPivot.kHomePosition);}));
    }*/



 }
  private void setupDashboard(){
    // A chooser for autonomous commands
    // Add commands to the autonomous command chooser
    m_autoChooser.addOption("Do Nothing", new WaitCommand(1));
    //m_autoChooser.addOption("BACKWARDS Auto Balance", Commands.getInstance().pidBalanceAuto);
    // m_autoChooser.addOption("(NOT DONE) Score Mid Cube", new WaitCommand(1)); //TODO
    m_autoChooser.addOption("Drive Backwards", new DriveTimedCommand(2,-0.5,m_driveSubsystem));
    /*m_autochooser0.addOption("Wait Auto", m_complexAuto);
    m_autochooser0.addOption("Dropoff Auto", m_complexAuto);
    m_autochooser0.addOption("Move Auto", m_complexAuto);
    m_autochooser0.addOption("Balance Auto", m_complexAuto);

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

    m_drivechooser.addOption("Twist Turn (default)",defaultDriveCommand);
    m_drivechooser.addOption("Arcade Turn (richard)", richardDriveCommand);
    m_drivechooser.addOption("Curvature Drive", curvatureDriveCommand);

    SmartDashboard.putData("Control Setup", m_drivechooser);*/
    SmartDashboard.putData("Autonomous", m_autoChooser);

  }


  public double getGyroReading() {
    m_gyro.getAngle();
    m_gyro.getXComplementaryAngle();
    m_gyro.getYComplementaryAngle();
    return 0.0;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new DriveTimedCommand(2,-0.5,m_driveSubsystem);
    //return Commands.getInstance().drive_backwards;
    return this.m_autoChooser.getSelected();
    //return new SequentialCommandGroup(this.m_autochooser0.getSelected(), this.m_autochooser1.getSelected(), this.m_autochooser2.getSelected(), this.m_autochooser3.getSelected());
  }
}

