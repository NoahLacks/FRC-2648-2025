package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    protected SparkMax elevatorMotor1;
    protected SparkMax elevatorMotor2;

    //private SparkClosedLoopController elevatorClosedLoop;

    protected RelativeEncoder encoder;

    private DigitalInput bottomLimitSwitch;

    private ProfiledPIDController pidController;

    private ElevatorFeedforward feedForward;
    
    public Elevator() {
        elevatorMotor1 = new SparkMax(
            ElevatorConstants.kElevatorMotor1ID, 
            MotorType.kBrushless
        );
        
        elevatorMotor2 = new SparkMax(
            ElevatorConstants.kElevatorMotor2ID,
            MotorType.kBrushless
        );

        elevatorMotor1.configure(
            ElevatorConstants.motorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        elevatorMotor2.configure(
            ElevatorConstants.motorConfig.follow(ElevatorConstants.kElevatorMotor1ID), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        //elevatorClosedLoop = elevatorMotor1.getClosedLoopController();

        encoder = elevatorMotor1.getEncoder();

        bottomLimitSwitch = new DigitalInput(
            ElevatorConstants.kBottomLimitSwitchID
        );

        pidController = new ProfiledPIDController(
            ElevatorConstants.kPositionControllerP, 
            ElevatorConstants.kPositionControllerI,
            ElevatorConstants.kPositionControllerD,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxVelocity, 
                ElevatorConstants.kMaxAcceleration
            )
        );
        pidController.setTolerance(ElevatorConstants.kAllowedError);

        feedForward = new ElevatorFeedforward(
            ElevatorConstants.kFeedForwardS, 
            ElevatorConstants.kFeedForwardG, 
            ElevatorConstants.kFeedForwardV
        );
    }

    @Override
    public void periodic() {
        if (!getBottomLimitSwitch()) {
            encoder.setPosition(0);
        }
    }

    /**
     * Returns whether or not the motion is safe relative to the encoder's current position
     * and the elevator brace position
     * 
     * @return Is the motion safe
     */
    public boolean isMotionSafe() {
        return isMotionSafe(getEncoderPosition());
    }

    /**
     * Returns whether or not the motion is safe relative to some target position and the elevator
     * brace position
     * 
     * @param motionTarget The target position to determine the safety of
     * @return Is the motion safe
     */
    public boolean isMotionSafe(double motionTarget) {
        return motionTarget > ElevatorConstants.kBracePosition;
    }
    
    /**
     * A manual translation command that uses feed forward calculation to maintain position
     * 
     * @param speed The speed at which the elevator translates
     * @return Sets motor voltage to translate the elevator and maintain position
     */
    public Command runManualElevator(DoubleSupplier speed) {
        return run(() -> {
            elevatorMotor1.set(
                speed.getAsDouble()
            );
        });
    }

    public Command maintainPosition() {
        return run(() -> {
            elevatorMotor1.setVoltage(feedForward.calculate(0));
        });
    }

    /**
     * Moves the elevator to a target destination (setpoint). 
     * 
     * @param setpoint Target destination of the subsystem
     * @param timeout Time to achieve the setpoint before quitting
     * @return Sets motor voltage to achieve the target destination
     */
    public Command goToSetpoint(DoubleSupplier setpoint) {
        double clampedSetpoint = MathUtil.clamp(
            setpoint.getAsDouble(), 
            0, 
            ElevatorConstants.kMaxHeight
        );

        return run(() -> {
            /* 
            if(pidController.atGoal()) {
                elevatorMotor1.setVoltage(feedForward.calculate(0));
            } else {
*/
                elevatorMotor1.setVoltage(
                    pidController.calculate(
                        encoder.getPosition(),
                        clampedSetpoint
                    ) + feedForward.calculate(pidController.getSetpoint().velocity)
                );
            
            
            /*elevatorClosedLoop.setReference(
                clampedSetpoint,
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                feedForward.calculate(0)
            );*/
            
        });

    }

    /**
     * Returns the current encoder position
     * 
     * @return Current encoder position
     */
    public double getEncoderPosition() {
        return encoder.getPosition();
    }


    /**
     * Returns the value of the bottom limit switch on the elevator (false = disabled, true = enabled)
     * 
     * @return The value of bottomLimitSwitch
     */
    public boolean getBottomLimitSwitch() {
        return bottomLimitSwitch.get();
    }

    public double getMotor1() {
        return elevatorMotor1.getOutputCurrent();
    }

    public double getMotor2() {
        return elevatorMotor2.getOutputCurrent();
    }
}