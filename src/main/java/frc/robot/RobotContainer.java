// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmPivot;
import frc.robot.commands.AutoBalancingPIDCommand;
import frc.robot.commands.DriveTimedCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.kauailabs.navx.frc.AHRS;

import java.util.Map;

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
  //public final ArmPivotSubsystem m_armPivot=null;// = new ArmPivotSubsystem();
  public final DumbArmPivotSubsystem m_dumbPivot = new DumbArmPivotSubsystem();

  public final DumbArmWristSubsystem m_dumbWrist = new DumbArmWristSubsystem();
  public final ArmElevatorSubsystem m_elevator = new ArmElevatorSubsystem();
  private boolean rumbling = true; // Is this needed?
  public final ZipMotorSubsystem m_zipMotor = new ZipMotorSubsystem();
  public final ZipPneumaticsSubsystem m_zipPneumatics = new ZipPneumaticsSubsystem();
  // ADIS Gyro
  //public ADIS16470_IMU m_gyro = new ADIS16470_IMU();


  public Controls m_controls = Controls.getInstance();
  UsbCamera camera1;
  UsbCamera camera2;

  //public frc.robot.commands.Commands m_commands = frc.robot.commands.Commands.getInstance();
  public static enum ArmStatuses {
    HOME,
    INTAKE,
    MID,
    HIGH_INTAKE,
    HIGH
  }
  public static ArmStatuses m_armStatus = ArmStatuses.HOME;
  public static Map<ArmStatuses,Double> m_wristPositions = Map.ofEntries(
          Map.entry(ArmStatuses.HOME,Constants.ArmWrist.kWHomePosition),
          Map.entry(ArmStatuses.INTAKE,Constants.ArmWrist.kWIntakePosition),
          Map.entry(ArmStatuses.MID,Constants.ArmWrist.kWMidScoringPosition),
          Map.entry(ArmStatuses.HIGH_INTAKE,Constants.ArmWrist.kWHighIntakePosition),
          Map.entry(ArmStatuses.HIGH,Constants.ArmWrist.kWHighScoringPosition)
  );

  public static Map<ArmStatuses,Double> m_wristSuckPositions = Map.ofEntries(
          Map.entry(ArmStatuses.HOME,Constants.ArmWrist.kWHomePosition),
          Map.entry(ArmStatuses.INTAKE,Constants.ArmWrist.kWCarryPosition),
          Map.entry(ArmStatuses.MID,Constants.ArmWrist.kWCarryPosition),
          Map.entry(ArmStatuses.HIGH_INTAKE,Constants.ArmWrist.kWCarryPosition),
          Map.entry(ArmStatuses.HIGH,Constants.ArmWrist.kWCarryPosition)
  );

  //private final Solenoid fake;
  
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  //SendableChooser<Command> m_autochooser1 = new SendableChooser<>();
  //SendableChooser<Command> m_autochooser2 = new SendableChooser<>();
  //SendableChooser<Command> m_autochooser3 = new SendableChooser<>();

  SendableChooser<RunCommand> m_drivechooser = new SendableChooser<>();

  public Command getNewMidDropoff(){
    return new SequentialCommandGroup(
            new InstantCommand(()->m_dumbPivot.setPos(Constants.ArmPivot.kMidScoringPosition),m_dumbPivot),
            new WaitCommand(1),
            new InstantCommand(()->m_dumbWrist.setPosition(Constants.ArmWrist.kWMidScoringPosition),m_dumbWrist),
            new WaitCommand(1),
            new InstantCommand(m_gripper::openGripper, m_gripper),
            new WaitCommand(1),
            new InstantCommand(()->m_dumbWrist.setPosition(Constants.ArmPivot.kHomePosition),m_dumbWrist),
            new WaitCommand(1),
            new InstantCommand(()->m_dumbPivot.setPos(Constants.ArmWrist.kWHomePosition),m_dumbPivot)
    );
  }

  public Command getNewHighDropoff(){
    return new SequentialCommandGroup(
            new InstantCommand(()->m_dumbPivot.setPos(ArmPivot.kHighScoringPosition),m_dumbPivot),
            new WaitCommand(1.5),
            new InstantCommand(m_elevator::Extend,m_elevator),
            new WaitCommand(1),
            new InstantCommand(()->m_dumbWrist.setPosition(Constants.ArmWrist.kWHighScoringPosition),m_dumbWrist),
            new WaitCommand(1),
            new InstantCommand(m_gripper::openGripper, m_gripper),
            new WaitCommand(.5),
            new InstantCommand(()->m_dumbWrist.setPosition(Constants.ArmWrist.kWHomePosition),m_dumbWrist),
            new WaitCommand(.5),
            new InstantCommand(m_elevator::Retract,m_elevator),
            new WaitCommand(1),
            new InstantCommand(()->m_dumbPivot.setPos(Constants.ArmWrist.kWHomePosition),m_dumbPivot)
    );
  }
  public final Command pidBalanceAuto = new SequentialCommandGroup(
          new DriveTimedCommand(1.5,-0.5,m_driveSubsystem),
          new AutoBalancingPIDCommand(m_driveSubsystem)
  );

  private final SequentialCommandGroup highScoreDriveBackwards = new SequentialCommandGroup(
          getNewHighDropoff(),
          new DriveTimedCommand(3,-0.5,m_driveSubsystem)
  );

  private final SequentialCommandGroup highScoreBalance = new SequentialCommandGroup(
          getNewHighDropoff(),
          new DriveTimedCommand(1.5,-0.5,m_driveSubsystem),
          new AutoBalancingPIDCommand(m_driveSubsystem)
  );

  private final RunCommand defaultDriveCommand = new RunCommand(()->{
    m_driveSubsystem.driveArcade(-m_controls.getThrottledY(), -m_controls.getThrottledTwist());
  }, m_driveSubsystem);

  private final RunCommand richardDriveCommand = new RunCommand(()->{
    m_driveSubsystem.driveArcade(-m_controls.getThrottledY(), m_controls.getThrottledX());
  }, m_driveSubsystem);

  private final RunCommand curvatureDriveCommand = new RunCommand(()->{
    m_driveSubsystem.driveCurvature(-m_controls.getThrottledY(), m_controls.getThrottledTwist(), m_controls.m_driverJoystick.button(11).getAsBoolean());
  }, m_driveSubsystem);

  private final SequentialCommandGroup rumbleCommandGroup = new SequentialCommandGroup(
          new InstantCommand(()-> m_controls.m_secondaryDriverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5)),
          new WaitCommand(.5),
          new InstantCommand(()-> m_controls.m_secondaryDriverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
  );
  private final SequentialCommandGroup shortRumbleCommandGroup = new SequentialCommandGroup(
          new InstantCommand(()-> m_controls.m_secondaryDriverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5)),
          new WaitCommand(.1),
          new InstantCommand(()-> m_controls.m_secondaryDriverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
  );
  




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

    m_dumbPivot.setDefaultCommand(new RunCommand(() -> {

      if(m_dumbPivot.changePos(m_controls.deadband(m_controls.m_secondaryDriverController.getLeftY())*-0.5)){
          CommandScheduler.getInstance().schedule(rumbleCommandGroup);
      }

    },m_dumbPivot));

    m_dumbWrist.setDefaultCommand(new RunCommand(() -> {
      if(m_dumbWrist.changePosition(m_controls.deadband(m_controls.m_secondaryDriverController.getRightY())*0.5)){
        CommandScheduler.getInstance().schedule(rumbleCommandGroup);
      }
    },m_dumbWrist));
    /*m_driveSubsystem.setDefaultCommand(new RunCommand(()->{
      m_driveSubsystem.driveArcade(-m_driverJoystick.getY(), -m_driverJoystick.getTwist());
    }, //TODO: Rotation throttle?
    m_driveSubsystem));*/

    // Aaron was here :D
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    camera1.setFPS(15);
    camera2.setFPS(15);
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
    /*m_controls.switchDriveButton.onTrue(new InstantCommand(()->{
      CommandScheduler.getInstance().cancel(m_driveSubsystem.getDefaultCommand());
      this.m_driveSubsystem.setDefaultCommand(this.m_drivechooser.getSelected());
      System.out.println(this.m_drivechooser.getSelected().getName());
    }));*/

    System.out.println("Configured Button Bindings");
    m_controls.gripperButton.onTrue(new InstantCommand(m_gripper::toggleGripper,m_gripper));
    m_controls.extendButton.onTrue(new InstantCommand(() -> {
      if(m_dumbPivot.m_encoder.getPosition() < ArmPivot.kSafeExtendLimit){
        m_elevator.toggle();
      } else {
        System.out.println("ARM IS TOO LOW!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        CommandScheduler.getInstance().schedule(rumbleCommandGroup);
      }
    },m_elevator));
    m_controls.highScoringPresetButton.onTrue(new InstantCommand(()->{
      m_dumbPivot.setPos(ArmPivot.kHighScoringPosition);
      m_armStatus = ArmStatuses.HIGH;
      },m_dumbPivot));
    m_controls.midScoringPresetButton.onTrue(new InstantCommand(()->{
      m_dumbPivot.setPos(ArmPivot.kMidScoringPosition);
      m_armStatus = ArmStatuses.MID;
      },m_dumbPivot));
    m_controls.highIntakePresetButton.onTrue(new InstantCommand(()->{
      m_dumbPivot.setPos(ArmPivot.kHighIntakePosition);
      m_armStatus = ArmStatuses.HIGH_INTAKE;
      },m_dumbPivot));
    m_controls.intakePresetButton.onTrue(new InstantCommand(()->{
      if(m_elevator.getExtended()){
        m_elevator.Retract();
      }
      m_dumbPivot.setPos(ArmPivot.kIntakePosition);
      m_armStatus = ArmStatuses.INTAKE;
      CommandScheduler.getInstance().schedule(shortRumbleCommandGroup);
      },m_dumbPivot,m_elevator));


    //Zipline arm and movement code

    //Toggling the zipline arm Solenoid direction when button is pressed
    //If this does not work, instead of using toggleOnTrue, use a similar method as used on lines 288-300 (the 22 lines below the todo make sure the elevator is retracted thingy)
    m_controls.armToggleButton.toggleOnTrue(new InstantCommand(()->{
      m_zipPneumatics.toggle();
    }, m_zipPneumatics));
    //Making the zipline go when zipGoButton (2) is pressed
    m_controls.zipGoButton.toggleOnTrue(new InstantCommand(()->{
      m_zipMotor.go();
    }, m_zipMotor));
    //Making the zipline stop when zipGoButton (2) stops being pressed
    m_controls.zipGoButton.toggleOnFalse(new InstantCommand(()->{
      m_zipMotor.notGo();
    }, m_zipMotor));

    //TESTING
    // m_controls.zipGoButton.onTrue(new InstantCommand(()->{
    //   m_zipMotor.go();
    // }, m_zipMotor));


    // TODO MAKE SURE THAT ELEVATOR IS RETRACTED HERE
    m_controls.homePresetButton.onTrue(new InstantCommand(()->{
      if(m_elevator.getExtended()){
        m_elevator.Retract();
      }
      m_dumbPivot.setPos(ArmPivot.kHomePosition);
      m_armStatus = ArmStatuses.HOME;
      CommandScheduler.getInstance().schedule(shortRumbleCommandGroup);
      },m_dumbPivot,m_elevator));
    m_controls.wristDeployButton.onTrue(new ConditionalCommand(
            new InstantCommand(()->{m_dumbWrist.setPosition(m_wristSuckPositions.get(m_armStatus));},m_dumbWrist),
            new InstantCommand(()->{m_dumbWrist.setPosition(m_wristPositions.get(m_armStatus));},m_dumbWrist),
            m_dumbWrist::checkDeployed
    ));
    m_controls.CubeButton.onTrue(new InstantCommand(()->SmartDashboard.putString(Constants.DashboardStrings.matrix_mode_str,"cube")));
    m_controls.ConeButton.onTrue(new InstantCommand(()->SmartDashboard.putString(Constants.DashboardStrings.matrix_mode_str,"cone")));

    
    //Zipline Controls TODO: finish this
    //m_controls.armToggleButton.onTrue()
    
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
    // m_autoChooser.addOption("(NOT DONE) Score Mid Cube", new WaitCommand(1));
    m_autoChooser.addOption("Drive Backwards", new DriveTimedCommand(2,-0.5,m_driveSubsystem));
    m_autoChooser.addOption("Dropoff Mid",getNewMidDropoff());
    m_autoChooser.addOption("Dropoff HIGH",getNewHighDropoff());
    m_autoChooser.addOption("Auto Balance Backwards",pidBalanceAuto);
    m_autoChooser.addOption("Score Retreat",highScoreDriveBackwards);
    m_autoChooser.addOption("Score then Balance",highScoreBalance);
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

