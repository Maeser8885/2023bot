package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class DumbArmPivotSubsystem extends SubsystemBase {
    private final CANSparkMax m_LPIVOTneo = new CANSparkMax(Constants.MotorConstants.kLARMSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_RPIVOTneo = new CANSparkMax(Constants.MotorConstants.kRARMSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);
    public final RelativeEncoder m_encoder;
    private SparkMaxPIDController m_controller;
    private double m_setpoint;
    private double m_prevSetpoint;

    public DumbArmPivotSubsystem() {
        m_RPIVOTneo.follow(m_LPIVOTneo, true);
        m_encoder = m_LPIVOTneo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        m_controller = m_LPIVOTneo.getPIDController();
        m_RPIVOTneo.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_RPIVOTneo.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 3.0f);
        PIDGains.setSparkMaxGains(m_controller, Constants.ArmPivot.kPositionPIDGains);
        m_setpoint = 0.0;

    }

    public void move(double speed){
        m_LPIVOTneo.set(speed);
    }
    public void changePos(double amount){
        if (m_setpoint + amount >= 0.0){
            m_setpoint = 0.0;
        } else {
            m_setpoint += amount;
        }
    }

    public void setPos(double pos){
        m_setpoint = pos;
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        if (m_setpoint != m_prevSetpoint) {
            m_controller.setReference(m_setpoint, CANSparkMax.ControlType.kPosition);
        }
        m_prevSetpoint = m_setpoint;

        SmartDashboard.putNumber("setpoint", m_setpoint);
        SmartDashboard.putNumber("Current arm val", m_encoder.getPosition());
    }
}
