package frc.robot.constants;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class ManipulatorPivotConstants {
    public static final int kArmMotorID = 1;

    public static final int kMotorCurrentMax = 40;

    public static final double kPivotConversion = 12.0/60.0 * 20.0/60.0 * 12.0/28.0;

    public static final double kPivotMaxVelocity = 0;

    public static final double kPositionalP = 0;
    public static final double kPositionalI = 0;
    public static final double kPositionalD = 0;
    public static final double kPositionalTolerance = Units.degreesToRadians(3.0);

    public static final double kFeedForwardS = 0;
    public static final double kFeedForwardG = 0.41; // calculated value
    public static final double kFeedForwardV = 0.68; //calculated value

    public static final double kFFGravityOffset = Units.degreesToRadians(-135.0);

    public static final double kMaxAcceleration = Units.degreesToRadians(1000.0); // degrees per second^2 calculated max = 2100
    public static final double kMaxVelocity = Units.degreesToRadians(100.0); // degrees per second calculated max = 168

    public static final double kCoralIntakePosition = Units.degreesToRadians(175.0);
    public static final double kL1Position = Units.degreesToRadians(0.0);
    public static final double kL2Position = Units.degreesToRadians(25.0);
    public static final double kL3Position = Units.degreesToRadians(60.0);
    public static final double kL4Position = Units.degreesToRadians(45.0);
    public static final double kL2AlgaePosition = Units.degreesToRadians(175.0);
    public static final double kL3AlgaePosition = Units.degreesToRadians(175.0);
    public static final double kProcesserPosition = Units.degreesToRadians(175.0);
    /**The closest position to the elevator brace without hitting it */
    public static final double kArmSafeStowPosition = Units.degreesToRadians(60.0);
    /**The forward rotation limit of the arm */
    public static final double kRotationLimit = Units.degreesToRadians(175.0);

    public static final double kSysIDRampRate = 1;
    public static final double kSysIDStepVolts = 7;
    public static final double kSysIDTimeout = 10;

    public static final SensorDirectionValue kSensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A NEW CONFIG

    public static final SysIdRoutine.Config kSysIDConfig = new Config(
        Volts.of(kSysIDRampRate).per(Second),
        Volts.of(kSysIDStepVolts),
        Seconds.of(kSysIDTimeout)
    );

    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    static {

        motorConfig
            .smartCurrentLimit(kMotorCurrentMax)
            .idleMode(kIdleMode);
        motorConfig.absoluteEncoder.positionConversionFactor(kPivotConversion);
        motorConfig.closedLoop
            .p(kPositionalP)
            .i(kPositionalI)
            .d(kPositionalD)
            .velocityFF(0.0); // keep at zero for position pid
        motorConfig.closedLoop.maxMotion
            .maxAcceleration(kMaxAcceleration)
            .maxVelocity(kMaxVelocity)
            .allowedClosedLoopError(kPositionalTolerance);
    }


}
