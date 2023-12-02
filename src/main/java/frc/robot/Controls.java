package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    public final static CommandJoystick m_driverJoystick =
            new CommandJoystick(Constants.OperatorConstants.kDriverControllerPort);

    // Triggers!

    // Primary Controls (DRIVING, ARM SETPOINTS)

    //public final Trigger switchDriveButton =
            //m_driverJoystick.button(Constants.BindingConstants.switchDrive);
    public final Trigger gripperButton = m_driverJoystick.button(1); //m_secondaryDriverController.a();
    public final Trigger extendButton = m_driverJoystick.button(3).and(m_driverJoystick.button(5)); //m_secondaryDriverController.rightBumper().and(m_secondaryDriverController.x());
            //m_secondaryDriverController.rightTrigger().and(m_secondaryDriverController.leftTrigger());
    public final Trigger midScoringPresetButton =
            m_driverJoystick.button(10);
    public final Trigger highScoringPresetButton =
            m_driverJoystick.button(8);
    public final Trigger highIntakePresetButton =
            m_driverJoystick.button(9);
    public final Trigger intakePresetButton =
            m_driverJoystick.button(11);
    public final Trigger homePresetButton =
            m_driverJoystick.button(12);
    public final Trigger wristDeployButton =
            m_driverJoystick.button(7);
            
    public final Trigger armToggleButton =
            m_driverJoystick.button(4).and(m_driverJoystick.button(6));
    public final Trigger zipGoButton =
            m_driverJoystick.button(2);
    
    // Secondary Driver Control (LED AND MANUAL)
    public final Trigger CubeButton = m_secondaryDriverController.x();
    public final Trigger ConeButton = m_secondaryDriverController.y();

    public double deadband(double input){
        if (Math.abs(input) < 0.1){
            return 0;
        }
        return input;
    }

    public double getThrottledY(){
        return m_driverJoystick.getY() * adjustThrottle(m_driverJoystick.getThrottle());
    }

    public double getThrottledTwist(){
        return 0.5 * m_driverJoystick.getTwist() * adjustThrottle(m_driverJoystick.getThrottle());
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
