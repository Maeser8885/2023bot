package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.ZiplineConstants;

public class ZiplinePneumaticsSubsystem extends SubsystemBase {
    
    public Compressor armCompressor = new Compressor(ZiplineConstants.kZipArmModule, PneumaticsModuleType.CTREPCM);
    public Solenoid zipExtender = new Solenoid(PneumaticsModuleType.CTREPCM, ZiplineConstants.kZipSolenoidModule);

    public ZiplinePneumaticsSubsystem() {
        Retract();
    }

    public void Extend() {
        zipExtender.set(ZiplineConstants.go);
    }

    public boolean getExtended() {
        return zipExtender.get();
    }

    public void Retract() {
        zipExtender.set(ZiplineConstants.notGo);
    }

    public void toggle() {
        zipExtender.toggle();
    }
}