package frc.robot.constants;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class ElevatorConstants {
    public static final int kElevatorMotor1ID = 8;
    public static final int kElevatorMotor2ID = 6;

    public static final int kBottomLimitSwitchID = 0;

    // 60/11 gearing multiplied by circumference of sprocket multiplied by 2 for carriage position
    public static final double kEncoderConversionFactor = 11.0/60.0 * (22.0*0.25) * 2.0;

    public static final int kCurrentLimit = 40;

    public static final double kPositionControllerP = 3; //
    public static final double kPositionControllerI = 0;
    public static final double kPositionControllerD = 0.1;//0.35
     
    public static final double kAllowedError = 0.75;

    public static final double kFeedForwardS = 0;
    public static final double kFeedForwardG = .7;//1; // calculated value  .6
    public static final double kFeedForwardV = 0.12; // calculated value  .12 INCHES NOT METERS

    public static final double kMaxVelocity = 120.0; // 120 inches per second (COOKING) calculated max is 184 in/s
    public static final double kMaxAcceleration = 240; // 400 inches per second^2 (also COOKING) calculated max is 600 in/s^2

    public static final double kCoralIntakePosition = 0;
    public static final double kL1Position = 0;
    public static final double kL2Position = 14.5;
    public static final double kL3Position = 29.0;
    public static final double kL4Position = 53.0;
    public static final double kL4TransitionPosition = 40.0;
    public static final double kL2AlgaePosition = 22.0;
    public static final double kL3AlgaePosition = 39.0;
    public static final double kProcessorPosition = 4.0;
    /**The position of the top of the elevator brace */
    public static final double kBracePosition = 5.5;
    public static final double kMaxHeight = 47.5; //actual is 53

    // 1, 7, 10 are the defaults for these, change as necessary
    public static final double kSysIDRampRate = .25;
    public static final double kSysIDStepVolts = 3;
    public static final double kSysIDTimeout = 10;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A NEW CONFIGURATION ITEM
    
    public static final SysIdRoutine.Config kSysIDConfig = new Config(
        Volts.of(kSysIDRampRate).per(Second), 
        Volts.of(kSysIDStepVolts), 
        Seconds.of(kSysIDTimeout)
    );

    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();
    
    static {
        motorConfig
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(kIdleMode)
            .inverted(true);
        motorConfig.encoder
            .positionConversionFactor(kEncoderConversionFactor)
            .velocityConversionFactor(kEncoderConversionFactor);
        /*motorConfig.closedLoop
            .p(kPositionControllerP)
            .i(kPositionControllerI)
            .d(kPositionControllerD)
            .velocityFF(0.0); // keep at zero for position pid
        motorConfig.closedLoop.maxMotion
            .maxAcceleration(kMaxAcceleration)
            .maxVelocity(kMaxVelocity)
            .allowedClosedLoopError(kAllowedError);*/
    }

    public static final SparkMaxConfig motorConfig2 = new SparkMaxConfig();

    static {
        motorConfig2
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(kIdleMode)
            .inverted(false);
        motorConfig.encoder
            .positionConversionFactor(kEncoderConversionFactor);  
    }
}
