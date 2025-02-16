package frc.robot.subsystems.sysid;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.InchesPerSecond;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorSysID extends Elevator {
    private MutVoltage appliedVoltage = Volts.mutable(0);;

    private MutDistance elevatorPosition = Inches.mutable(0);;

    private MutLinearVelocity elevatorVelocity = InchesPerSecond.mutable(0);

    private SysIdRoutine routine;

    public ElevatorSysID() {
        super();

        routine = new SysIdRoutine(
            ElevatorConstants.kSysIDConfig,
            new SysIdRoutine.Mechanism(
                elevatorMotor1::setVoltage,
                (log) -> {
                    log.motor("elevatorMotor1")
                        .voltage(appliedVoltage.mut_replace(
                            elevatorMotor1.get() * RobotController.getBatteryVoltage(), Volts
                        ))
                        .linearPosition(elevatorPosition.mut_replace(
                            encoder.getPosition(), Inches
                        ))
                        .linearVelocity(elevatorVelocity.mut_replace(
                            encoder.getVelocity(), InchesPerSecond
                        ));
                }, 
                this
            )
        );
    }

    @Override
    public void periodic() {

    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
