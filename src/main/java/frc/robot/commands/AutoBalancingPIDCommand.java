package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;


public class AutoBalancingPIDCommand extends CommandBase {
    // Thanks RI3D GOFIRST-Robotics!
    // github.com/GOFIRST-Robotics/Ri3D-2023
    private final DriveSubsystem driveSubsystem;
    //private final ADIS16470_IMU gyro = RobotContainer.getInstance().m_gyro;
    private final AHRS navX;

    private double error;
    private double currentAngle;
    private double drivePower;
    private Timer runTimer = new Timer();
    private double currentTime;
    private double prev_time = 0;
    private double prev_error = 0;
    private Timer end_Timer = new Timer();
    private boolean timerRunning = false;

    public AutoBalancingPIDCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.navX = driveSubsystem.navX;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        end_Timer.reset();
        timerRunning = false;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        this.currentAngle = navX.getRoll();

        error = Constants.balanceGoal - currentAngle;
        drivePower = Math.min(Constants.balanceKP * error, 1);
// Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        //drivePower *= Constants.balanceBackwardsXTRAPOWA;

        // Limit the max power
        if (Math.abs(drivePower) > 0.4) {
            drivePower = Math.copySign(0.4, drivePower);
        }

        driveSubsystem.driveArcade(drivePower,0);

        // Debugging Print Statments
        System.out.println("Current Angle: " + currentAngle);
        System.out.println("Error " + error);
        System.out.println("Drive Power: " + drivePower);

    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        if (Math.abs(error) < Constants.balanceThreshold){
            if (timerRunning){
                System.out.println("Timer:" + end_Timer.get());
                if(end_Timer.get() > 2.0){
                    return true;
                } else {
                    return false;
                }
            } else {
                timerRunning = true;
                end_Timer.start();
                return false;
            }
        }
        if (timerRunning){
            end_Timer.stop();
            end_Timer.reset();
            timerRunning = false;
        }
        return false; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    }
    //TODO Timer if in this state

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveArcade(0,0);
    }
}
