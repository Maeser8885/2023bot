package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Commands;

public class Controls {
    private static Controls instance;

    public static synchronized Controls getInstance(){
        if (instance == null){
            instance = new Controls();
        }
        return instance;
    }
    //public boolean m_debounce;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public final CommandXboxController m_secondaryDriverController =
            new CommandXboxController(Constants.OperatorConstants.kSecondaryDriverControllerPort);
    public final CommandJoystick m_driverJoystick =
            new CommandJoystick(Constants.OperatorConstants.kDriverControllerPort);

    // Triggers!
    public final Trigger switchDriveButton =
            m_driverJoystick.button(Constants.BindingConstants.switchDrive);
    public final Trigger gripperButton =
            m_secondaryDriverController.rightTrigger().and(m_secondaryDriverController.leftTrigger());
    public final Trigger lowScoringButton =
            m_secondaryDriverController.rightBumper().and(m_secondaryDriverController.x());
    public final Trigger highScoringButton =
            m_secondaryDriverController.a();
    public final Trigger highIntakeButton =
            m_secondaryDriverController.rightBumper().and(m_secondaryDriverController.b());
    public final Trigger IntakeButton =
            m_secondaryDriverController.rightBumper().and(m_secondaryDriverController.y());
    public final Trigger HomeButton =
            m_secondaryDriverController.leftBumper().and(m_secondaryDriverController.x());
    public final Trigger WristPivotStick =
            m_secondaryDriverController.leftStick();

    public double getThrottledY(){
        return m_driverJoystick.getY() * adjustThrottle(m_driverJoystick.getThrottle());
    }

    public double getThrottledTwist(){
        return m_driverJoystick.getTwist() * adjustThrottle(m_driverJoystick.getThrottle());
    }

    public double getThrottledX(){
        return m_driverJoystick.getX() * adjustThrottle(m_driverJoystick.getThrottle());
    }


    private double adjustThrottle(double throttle) {
        return throttle/2 +.5;
    }

    /*public int leftJoystickFlick() {
        if (m_debounce) return 0;
        if (m_secondaryDriverController.getLeftY() < 120) return 0;
        //stuff
        return Integer.signum((int)m_secondaryDriverController.getLeftY());
    }*/

}
