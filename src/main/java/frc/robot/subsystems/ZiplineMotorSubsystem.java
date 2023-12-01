package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ZiplineConstants;

public class ZiplineMotorSubsystem extends SubsystemBase {
    CANSparkMax zipMotor = new CANSparkMax(ZiplineConstants.kMotorPort, MotorType.kBrushless);
    
    public void go(){
        zipMotor.set(1.0);
    }

    public void notGo(){
        zipMotor.set(0.0);
    }
}
