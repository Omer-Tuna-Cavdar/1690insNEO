package frc.robot;

import java.io.File;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Subsytems.Intake;
import frc.robot.Subsytems.Pivot;
import frc.robot.Subsytems.Shooter;
import frc.robot.Subsytems.SwerveSubsystem;
import frc.robot.Subsytems.Vision;
import swervelib.math.Matter;

public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
      // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }
  public static final class IntakeConstants {
    public static final int kIntakeIrBeamBreakPort = 1;
    public static final int kShooterRId = 41;
    public static final boolean INTAKE_ROLLER_INVERTED = false;
    public static final double Intake_Roller_Speed = 0.7;
    public static final double Intake_Roller_Speed_ForAmp = 0.7;
  
    
  }
  public static final class ShooterConstans {
    public static final double kShooterP = 0.7;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    public static final double kShooterPositionTolerance = 3.0;
    public static final double kShooterVelocityTolerance = 3.0;
    public static final double SHOOTER_TARGET_RPM = 4000.0;
    public static final int kShooterLId = 31;
    public static final int kShooterRId= 32;
    public static final double SHOOTER_RPM_Scailing_Per_Meter = 2000;
	public static final double SHOOTER_RPM_FORAMP = 100;
    public static final double FEED_DURATION_SECONDS = 0.5;
  }
  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class Subsytems{
    public static final SwerveSubsystem SWERVE_SUBSYSTEM = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    public static final Vision vision= new Vision("Ordek"); 
    public static final Shooter shooter = new Shooter();
    public static final Intake intake = new Intake();
    public static final Pivot pivot = new Pivot();
  }
public static final class PivotConstants {
        public static final int kPivotMotor1CanId = 21;
        public static final int kPivotMotor2CanId = 22;
        public static final int kPivotIrBeamBreakPort = 0;
        public static final int kPivotCANCoderId = 23;
        public static final double kIntakePosition = 2;
        public static final double kAmpScoringPosition = 17.5;
        public static final double kSubwooferShotPosition = 9;
        public static final double kReverseShotPosition = 18.25;
        public static final double kStowPosition = 0.5;
        public static final double kEjectPosition = 10;
        public static final double kOuttakePosition = 5;
        public static final double kPivotErrorMargin = 0.2;
        public static final double kMinFeedPosition = 4;
        public static final double gearReduction = 1;
        public static final double kP = 0.7;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        public static final double kMinOutput = -0.7;
        public static final double kConversionFactor = 360.0 / 4096.0;
        public static double kMaxOutput = 0.7;
   
    }
    public static final class FieldConstants{
      public static final double SPEAKER_TARGET_HEIGHT = 2.11; // Speaker target height in meters
      public static final double AMP_TARGET_HEIGHT = 1.3558;   // Amp target height in meters
      public static final double SOURCE_TARGET_HEIGHT = 0.5338; // Source target height in meters
  
      // Constants for AprilTag heights in meters
      public static final double SPEAKER_TAG_HEIGHT = 0.5713;
      public static final double AMP_TAG_HEIGHT = 1.3558;
      public static final double SOURCE_TAG_HEIGHT = 0.5338;
  
      
    }



}