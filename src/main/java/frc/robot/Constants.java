// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Compressor;
import frc.lib.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryDriverControllerPort = 1;
    
  }
  // Used in Balancing 
  public static final int autoConst = 20;
  public static final int PCMCan = 6;

  public static class MotorConstants {
    // Used in DriveSubsystem,,..,.
    public static final int kFLSparkMax = 4;
    public static final int kFRSparkMax = 2;
    public static final int kBLSparkMax = 3;
    public static final int kBRSparkMax = 1;
    // Used in ArmPivotSubsystem
    public static final int kLARMSparkMax = 10;
    public static final int kRARMSparkMax = 11;
    // Used in ArmWristSubsystem
    public static final int kWRISTSparkMax = 12;
    // Used in GripperSubsystem
    public static final int kGRIPSparkMax = 13;
  }
  public static class BindingConstants {
    public static final int switchDrive = 7;
  }
  public static class ArmPivot { //Subject to Change.
    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);

    public static final double kArmGearRatio = 1.0 / (48.0 * 4.0); 
    public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
    public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
    public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

    public static final double kHomePosition = 0.0;
    public static final double kHighScoringPosition = 3.05;
    public static final double kLowScoringPosition = 3.05;
    public static final double kIntakePosition = 4.52;
    public static final double kHighIntakePosition = 2.95;
    //public static final double kFeederPosition = 2.95; Was this for high intake?? Oh well.
  }
  public static class ArmWrist { //Subject to Change.
    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);

    public static final double kArmGearRatio = 1.0 / (48.0 * 4.0); 
    public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
    public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
    public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

    public static final double kHomePosition = 0.0;
    public static final double kScoringPosition = 3.05;
    public static final double kIntakePosition = 4.52;
    public static final double kFeederPosition = 2.95;
  }
  public static class PneumaticConstants {
    // Used in ArmElevatorSubsystem
    public static final int kLforward = 1;
    public static final int kLreverse = 2;
    public static final int kRforward = 3;
    public static final int kRreverse = 4;
  }
  public static class AutoConstants {
    public static final double kAutoDriveDistanceInches = 5;
  }
}
