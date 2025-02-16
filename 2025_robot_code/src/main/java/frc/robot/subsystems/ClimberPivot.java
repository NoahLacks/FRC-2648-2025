package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberPivotConstants;

public class ClimberPivot extends SubsystemBase {
    private SparkMax pivotMotor;

    private RelativeEncoder neoEncoder;

    public ClimberPivot() {
        pivotMotor = new SparkMax(
            ClimberPivotConstants.kPivotMotorID, 
            MotorType.kBrushless
        );

        pivotMotor.configure(
            ClimberPivotConstants.motorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        neoEncoder = pivotMotor.getEncoder();
    }

    public Command runPivot(double speed) {
        return run(() -> {
            pivotMotor.set(speed);
        });
    }

    /**
     * Runs the climber until it is at setpoint
     * 
     * @param speed The speed at which the pivot runs
     * @param setpoint The target position of the climber
     * @return Sets the motor speed until at the target position
     */
    public Command climb(double setpoint, double speed) {
        return run(() -> {
            pivotMotor.set(speed);
        }).until(() -> neoEncoder.getPosition() >= setpoint);
    }

    public double getEncoderPosition() {
        return neoEncoder.getPosition();
    }
}