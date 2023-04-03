package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;
import frc.robot.Constants.Limits;

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
        m_controller.setOutputRange(-0.4,0.4);

    }

//    public void move(double speed){
//        m_LPIVOTneo.set(speed);
//    }
    public boolean changePos(double amount){ // return true if at limit
        Limits limit = checkInBounds(m_setpoint += amount);
        //System.out.println(limit);
        if (limit == Limits.MAX){
            m_setpoint = Constants.ArmPivot.kMaxPosition;
            return true;
        } else if (limit == Limits.MIN){
            m_setpoint = Constants.ArmPivot.kMinPosition;
            return true;
        } else {
            m_setpoint += amount;
            return false;
        }
    }

    private Limits checkInBounds(double point){
        // Less than max cuz arm points are always negative
        if (point < Constants.ArmPivot.kMaxPosition){
            return Limits.MAX;
        } else if (point > Constants.ArmPivot.kMinPosition){
            return Limits.MIN;
        }
        return Limits.NONE;
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
