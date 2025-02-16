package frc.robot.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingFactor = kWheelDiameterMeters * Math.PI / kDrivingMotorReduction;
    public static final double kTurningFactor = 2 * Math.PI;
    public static final double kDrivingVelocityFeedForward = 1 / kDriveWheelFreeSpeedRps;
    
    public static final double kDriveP = .04;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveS = 0;
    public static final double kDriveV = kDrivingVelocityFeedForward;
    public static final double kDriveA = 0;
    
    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final int kDriveMotorStatorCurrentLimit = 100;
    public static final int kDriveMotorSupplyCurrentLimit = 65;
    public static final int kTurnMotorCurrentLimit = 20;

    public static final IdleMode kTurnIdleMode = IdleMode.kBrake;

    public static final InvertedValue kDriveInversionState = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kDriveIdleMode = NeutralModeValue.Brake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A NEW CONFIGURATION ITEM 

    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    public static final FeedbackConfigs kDriveFeedConfig = new FeedbackConfigs();
    public static final CurrentLimitsConfigs kDriveCurrentLimitConfig = new CurrentLimitsConfigs();
    public static final MotorOutputConfigs kDriveMotorConfig = new MotorOutputConfigs();
    public static final Slot0Configs kDriveSlot0Config = new Slot0Configs();

    static {
        kDriveFeedConfig.SensorToMechanismRatio = kDrivingMotorReduction;

        kDriveCurrentLimitConfig.StatorCurrentLimitEnable = true;
        kDriveCurrentLimitConfig.SupplyCurrentLimitEnable = true;
        kDriveCurrentLimitConfig.StatorCurrentLimit = kDriveMotorStatorCurrentLimit;
        kDriveCurrentLimitConfig.SupplyCurrentLimit = kDriveMotorSupplyCurrentLimit;

        kDriveMotorConfig.Inverted = kDriveInversionState;
        kDriveMotorConfig.NeutralMode = kDriveIdleMode;

        kDriveSlot0Config.kP = kDriveP;
        kDriveSlot0Config.kI = kDriveI;
        kDriveSlot0Config.kD = kDriveD;
        kDriveSlot0Config.kS = kDriveS;
        kDriveSlot0Config.kV = kDriveV;
        kDriveSlot0Config.kA = kDriveA;

        turningConfig
            .idleMode(kTurnIdleMode)
            .smartCurrentLimit(kTurnMotorCurrentLimit);
        turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(kTurningFactor) // radians
            .velocityConversionFactor(kTurningFactor / 60.0); // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(kTurnP, kTurnI, kTurnD)
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, kTurningFactor);
    }
}
