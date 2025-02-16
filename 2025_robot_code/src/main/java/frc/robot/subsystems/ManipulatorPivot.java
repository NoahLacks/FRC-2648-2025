package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorPivotConstants;

public class ManipulatorPivot extends SubsystemBase {
    protected SparkMax armMotor;

    private ArmFeedforward feedForward;

    private SparkClosedLoopController pivotClosedLoopController;
    private SparkAbsoluteEncoder absoluteEncoder;

    public ManipulatorPivot() {
        armMotor = new SparkMax(
            ManipulatorPivotConstants.kArmMotorID, 
            MotorType.kBrushless
        );

        armMotor.configure(
            ManipulatorPivotConstants.motorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        pivotClosedLoopController = armMotor.getClosedLoopController();
        absoluteEncoder = armMotor.getAbsoluteEncoder();
        
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
        return motionTarget > ManipulatorPivotConstants.kArmSafeStowPosition;
    }

    public Command setSpeed(DoubleSupplier speed) {
        return run(() -> {
            armMotor.set(speed.getAsDouble());
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
            pivotClosedLoopController.setReference(clampedSetpoint,
                SparkBase.ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot1,
                feedForward.calculate(absoluteEncoder.getPosition() + ManipulatorPivotConstants.kFFGravityOffset, absoluteEncoder.getVelocity()));
        });
    }

    /**
     * Returns the CANCoder's position in radians
     * 
     * @return CANCoder's position in radians
     */
    public double getEncoderPosition() {
        return absoluteEncoder.getPosition();
    }
    /**
     * Returns the CANCoder's velocity in radians per second
     * 
     * @return CANCoder's velocity in radians per second
     */
    public double getEncoderVelocity() {
        return absoluteEncoder.getVelocity();
    }
}
