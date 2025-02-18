package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorPivotConstants;

public class ManipulatorPivot extends SubsystemBase {
    protected SparkMax pivotMotor;

    private SparkAbsoluteEncoder encoder;

    private ArmFeedforward feedForward;

    private ProfiledPIDController pidController;

    public ManipulatorPivot() {
        pivotMotor = new SparkMax(
            ManipulatorPivotConstants.kPivotMotorID, 
            MotorType.kBrushless
        );

        pivotMotor.configure(
            ManipulatorPivotConstants.motorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
        
        encoder = pivotMotor.getAbsoluteEncoder();

        pidController = new ProfiledPIDController(
            ManipulatorPivotConstants.kPositionalP, 
            ManipulatorPivotConstants.kPositionalI, 
            ManipulatorPivotConstants.kPositionalD, 
            new TrapezoidProfile.Constraints(ManipulatorPivotConstants.kMaxVelocity, ManipulatorPivotConstants.kMaxAcceleration)
        );
        
        feedForward = new ArmFeedforward(
            ManipulatorPivotConstants.kFeedForwardS,
            ManipulatorPivotConstants.kFeedForwardG, 
            ManipulatorPivotConstants.kFeedForwardV
            );
    }

    /**
     * Returns whether or not the motion is safe relative to the encoder's current position
     * and the arm safe stow position
     * 
     * @return Is the motion safe
     */
    public boolean isMotionSafe() {
        return isMotionSafe(getEncoderPosition());
    }

    /**
     * Returns whether or not the motion is safe relative to some target position and the
     * arm safe stow position
     * 
     * @param motionTarget The target position to determine the safety of
     * @return Is the motion safe
     */
    public boolean isMotionSafe(double motionTarget) {
        return motionTarget > ManipulatorPivotConstants.kPivotSafeStowPosition;
    }

    /**
     * Manual ManipulatorPivot command that sets the motor based on speed
     * 
     * @param speed The speed to set the motor
     * @return A command that sets the motor speed
     */
    public Command runManualPivot(DoubleSupplier speed) {
        return run(() -> {
            pivotMotor.set(speed.getAsDouble());
        });
    }
    
    /**
     * Moves the arm to a target destination (setpoint)
     * 
     * @param setpoint Target destination of the subsystem
     * @param timeout Time to achieve the setpoint before quitting
     * @return Sets motor voltage to achieve the target destination
     */
    public Command goToSetpoint(double setpoint) {
        double clampedSetpoint = MathUtil.clamp(
            setpoint, 
            0, 
            ManipulatorPivotConstants.kRotationLimit
        );

        return run(() -> {
            pidController.reset(encoder.getPosition(), encoder.getVelocity());

            pivotMotor.setVoltage(
                pidController.calculate(
                    encoder.getPosition(), 
                    clampedSetpoint
                ) + feedForward.calculate(
                    encoder.getPosition(), 
                    0
                )
            );
        });
    }

    /**
     * Returns the encoder's position in radians
     * 
     * @return Encoder's position in radians
     */
    public double getEncoderPosition() {
        return encoder.getPosition();
    }
    /**
     * Returns the encoder's velocity in radians per second
     * 
     * @return Encoder's velocity in radians per second
     */
    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }
}
