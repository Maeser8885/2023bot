package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Commands {
    private static Commands instance;
    private final RobotContainer robotContainer = RobotContainer.getInstance();

    public static synchronized Commands getInstance(){
        if (instance == null){
            instance = new Commands();
        }
        return instance;
    }

    public final Command balanceAuto = new AutoBalancingCommand(robotContainer.m_driveSubsystem, robotContainer.m_gyro);
    // Literally just waits there, menacingly
    public final Command waitAuto = new WaitCommand(3);
    // Moves the robot a set amount.
    public final Command drive_backwards = new DriveDistance(12, -0.5, robotContainer.m_driveSubsystem);
    // Moves the robot to Scoring Area, drops the game piece.
    public final Command dropoffAuto = new AutoDropoffCommand(robotContainer.m_gripper);
    // placeholder command
    public final Command m_complexAuto = new RunCommand(()->{});


    public final Command pidBalanceAuto = new SequentialCommandGroup(
            new DriveTimedCommand(1.5,-0.5,robotContainer.m_driveSubsystem),
            new AutoBalancingPIDCommand(robotContainer.m_driveSubsystem)
    );

    public final Command dropOffAutoCubeMid = new SequentialCommandGroup(
            new InstantCommand(()->robotContainer.m_dumbPivot.setPos(Constants.ArmPivot.kMidScoringPosition),robotContainer.m_dumbPivot), //TODO THESE VALUES ARE RELATIVE TO PIECE NOT TO MOTOR
            new InstantCommand(()->robotContainer.m_dumbWrist.setPosition(Constants.ArmWrist.kWMidScoringPosition),robotContainer.m_dumbWrist), //TODO Test
            new InstantCommand(robotContainer.m_gripper::openGripper, robotContainer.m_gripper),
            new InstantCommand(()->robotContainer.m_dumbWrist.setPosition(Constants.ArmPivot.kHomePosition),robotContainer.m_dumbWrist),
            new InstantCommand(()->robotContainer.m_dumbPivot.setPos(Constants.ArmWrist.kWHomePosition),robotContainer.m_dumbPivot)
    );


    // MAKE COMMANDS HERE!!
    // Once on the Charging Station, it balances hopefully.

}
