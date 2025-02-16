package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
    private SparkMax manipulatorMotor;

    private DigitalInput coralBeamBreak;

    public Manipulator() {
        manipulatorMotor = new SparkMax(
            ManipulatorConstants.kManipulatorMotorID, 
            MotorType.kBrushless
        );

        manipulatorMotor.configure(
            ManipulatorConstants.motorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        coralBeamBreak = new DigitalInput(ManipulatorConstants.kCoralBeamBreakID);
    }

    /**
     * The default command for the manipulator that either stops the manipulator or slowly 
     * runs the manipulator to retain the algae
     * 
     * @return Returns a command that sets the speed of the motor
     */
    public Command defaultCommand() {
        return run(() -> {
            runUntilCollected(() -> 0.1);
            });
    }

    /**
     * Runs the manipulator at a set speed with the direction based on the coral parameter
     * 
     * @param speed The speed at which the manipulator runs
     * @param coral Is the manipulator manipulating a coral? (True = Coral, False = Algae)
     * @return Returns a command that sets the speed of the motor
     */
    public Command runManipulator(DoubleSupplier speed, boolean coral) {
        return run(() -> {
            manipulatorMotor.set(
                coral ? speed.getAsDouble() : speed.getAsDouble() * -1
            );
        });
    }

    /**
     * Runs the manipulator until either the algae or coral beam break reads true
     * 
     * @param speed The speed at which the manipulator is run
     * @param coral Is the object a coral? (True = Coral, False = Algae)
     * @return Returns a command that sets the speed of the motor
     */
    public Command runUntilCollected(DoubleSupplier speed) {
        return run(() -> {
            manipulatorMotor.set(
                speed.getAsDouble()
            );
        }).until(() -> !coralBeamBreak.get());
    }

    /**
     * Runs the manipulator in a way that will bring the coral to a reliable holding position
     * 
     * @return Returns a command that will position the coral to a known location
     */
    public Command indexCoral() {
        return run(() -> {
            runUntilCollected(() -> 0.5)
                .andThen(runManipulator(() -> .1, false))
                .until(() -> getCoralBeamBreak());
        });
    }

    public boolean getCoralBeamBreak() {
        return coralBeamBreak.get();
    }
}
