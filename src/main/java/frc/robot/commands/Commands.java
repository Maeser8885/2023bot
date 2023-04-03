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

    //public final Command balanceAuto = new AutoBalancingCommand(robotContainer.m_driveSubsystem, robotContainer.navX);
    // Literally just waits there, menacingly
    public final Command waitAuto = new WaitCommand(3);
    // Moves the robot a set amount.
    public final Command drive_backwards = new DriveDistance(12, -0.5, robotContainer.m_driveSubsystem);
    // Moves the robot to Scoring Area, drops the game piece.
    public final Command dropoffAuto = new AutoDropoffCommand(robotContainer.m_gripper);
    // placeholder command
    public final Command m_complexAuto = new RunCommand(()->{});







    // MAKE COMMANDS HERE!!
    // Once on the Charging Station, it balances hopefully.

}
