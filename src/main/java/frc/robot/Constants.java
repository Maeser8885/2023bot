// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  public static final double balanceGoal = 0; //TODO CHANGE ME
  public static final double balanceKP = 0.025; // TODO CHANGE ME .021
  public static final double balanceKD = 0.01;
  public static final double balanceThreshold = 1.0; // Maybe change?
  public static final double balanceBackwardsXTRAPOWA = 1.35; //TODO CHANGE ME
  public static final int PCMCan = 6;

  public static final int wheelDiameter = 6;

  public static class MotorConstants {
    // Used in DriveSubsystem,,..,.
    public static final int kFLSparkMax = 4;
    public static final int kFRSparkMax = 2;
    public static final int kBLSparkMax = 3;
    public static final int kBRSparkMax = 1;
    // Used in ArmPivotSubsystem
    public static final int kLARMSparkMax = 12;
    public static final int kRARMSparkMax = 11;
    // Used in ArmWristSubsystem
    public static final int kWRISTSparkMax = 13;
    // Used in GripperSubsystem
    public static final int kGRIPSparkMax = 14;
  }
  public static class BindingConstants {
    public static final int switchDrive = 7;
  }

  public static enum Limits {
    MAX,
    MIN,
    NONE
  }

  public static class DashboardStrings
  {
    public static final String matrix_mode_str = "matrix_mode";
  }
  public static class ArmPivot { //Subject to Change.
    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);

    public static final double kSafeExtendLimit = -25.0;
    public static final double kHomePosition = 0.0;
    public static final double kHighScoringPosition = -50.0;
    public static final double kMidScoringPosition = -40;
    public static final double kIntakePosition = -5.85;
    public static final double kHighIntakePosition = -38;

    public static final double kMaxPosition = -90.0;
    public static final double kMinPosition = 0.0;
    //public static final double kFeederPosition = 2.95; Was this for high intake?? Oh well.
  }
  public static class ArmWrist { //Subject to Change.
    public static final double kWHomePosition = 0.1;
    public static final double kWHighScoringPosition = 42.7;
    public static final double kWMidScoringPosition = 42.7;
    public static final double kWIntakePosition = 42.7;
    public static final double kWHighIntakePosition = 42.7;
    public static final double kWCarryPosition = 2.8;

    //public static final double kHomePosition = 0.0;
    public static final double kMaxPosition = 43.0;
    //public static final double kMinPosition = 0.0;
    //public static final double kSafePosition = -29.0;
    public static final int kCurrentLimit = 20;
    public static final PIDGains kPositionPIDGains = new PIDGains(0.1, 0.0, 0.0);

    public static final double kGearRatio = 1.0 / (48.0 * (6.0 / 2.0));
  }
  public static class PneumaticConstants {
    public static int kSecondaryPCM = 1;
    // Used in ArmElevatorSubsystem
    public static final int kL_PortForward = 4;
    public static final int kL_PortReverse = 5;

    public static final int kR_PortForward = 6;
    public static final int kR_PortReverse = 7;
    public static final int kGripForward = 1;
    public static final int kGripReverse = 0;
  }
  public static class AutoConstants {
    public static final double kAutoDriveDistanceInches = 5;
  }

  public static class ZiplineConstants{
    //TODO change numbers for everything after asking electrical
    //Compressor constants
    public static final int kZipArmForward = 0;
    public static final int kZipArmReverse = 0;
    public static final int kZipArmModule = 0;
    public static final int kZipSolenoidModule = 0;
    //Ty don't be mad at us this is all we have please.
    public static final boolean go = true;
    public static final boolean notGo = false;
    //Motor constants
    public static final int kMotorPort = 0;
    public static final double kSparkMaxRPM = 0.0;
  }
}


 /* public static final double kArmGearRatio = 1.0 / (48.0 * (62.0/16.0));
  public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
  public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
  public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
  public static final double kArmZeroCosineOffset = Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
  public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
  //public static final PIDGains kArmPositionGains = new PIDGains(0.1, 0.0, 0.0);
  public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

*/