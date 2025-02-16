package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberRollersConstants;

//TODO Figure out a way to detect if we're at the top of the cage
public class ClimberRollers extends SubsystemBase {
    private SparkMax rollerMotor;

    public ClimberRollers() {
        rollerMotor = new SparkMax(
            ClimberRollersConstants.kRollerMotorID, 
            MotorType.kBrushless
        );

        rollerMotor.configure(
            ClimberRollersConstants.motorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    /**
     * Runs the rollers at a set speed
     * 
     * @param speed The speed in which the roller runs
     * @return Runs the rollers at a set speed
     */
    public Command runRoller(double speed) {
        return run(() -> {
            rollerMotor.set(speed);
        });
    }
}
