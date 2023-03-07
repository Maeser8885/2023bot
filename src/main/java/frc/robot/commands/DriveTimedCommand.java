package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import org.ejml.data.FScalar;

public class DriveTimedCommand extends CommandBase {
    private final DriveSubsystem m_subsystem;
    private final double m_speed;
    private final double m_time;

    private final Timer m_timer = new Timer();

    public DriveTimedCommand(double time, double speed, DriveSubsystem subsystem) {
        m_time = time;
        m_speed = speed;
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.driveArcade(m_speed, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.driveArcade(0,0);
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.get() >= m_time;
    }

}
