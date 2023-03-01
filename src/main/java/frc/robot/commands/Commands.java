package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class Commands {
    private static Commands instance;
    private RobotContainer robotContainer = RobotContainer.getInstance();

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
    public final Command drive = new DriveDistance(1, 2, robotContainer.m_driveSubsystem);
    // Moves the robot to Scoring Area, drops the game piece.
    public final Command dropoffAuto = new AutoDropoffCommand(robotContainer.m_gripper);
    // placeholder command
    public final Command m_complexAuto = new RunCommand(()->{});

    // MAKE COMMANDS HERE!!
    // Once on the Charging Station, it balances hopefully.

}
