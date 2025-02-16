package frc.robot.constants;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class DrivetrainConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.PI;
    public static final double kFrontRightChassisAngularOffset = -Math.PI / 2;
    public static final double kBackLeftChassisAngularOffset = Math.PI / 2;
    public static final double kBackRightChassisAngularOffset = 0;

    // 1, 7, 10 is the default for these three values
    public static final double kSysIDDrivingRampRate = 1;
    public static final double kSysIDDrivingStepVolts = 7;
    public static final double kSysIDDrivingTimeout = 10;

    // 1, 7, 10 is the default for these three values
    public static final double kSysIDTurningRampRate = 1;
    public static final double kSysIDTurningStepVolts = 7;
    public static final double kSysIDTurningTimeout = 10;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 0;
    public static final int kRearLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 5;

    public static final boolean kGyroReversed = true;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A NEW CONFIGURATION ITEM

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SysIdRoutine.Config kSysIDDrivingConfig = new Config(
        Volts.of(kSysIDDrivingRampRate).per(Second), 
        Volts.of(kSysIDDrivingStepVolts), 
        Seconds.of(kSysIDDrivingTimeout)
    );

    public static final SysIdRoutine.Config kSysIDTurningConfig = new Config(
        Volts.of(kSysIDTurningRampRate).per(Second), 
        Volts.of(kSysIDTurningStepVolts), 
        Seconds.of(kSysIDTurningTimeout)
    );
  }
