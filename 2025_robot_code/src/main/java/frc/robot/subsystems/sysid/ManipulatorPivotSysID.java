package frc.robot.subsystems.sysid;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ManipulatorPivotConstants;
import frc.robot.subsystems.ManipulatorPivot;

public class ManipulatorPivotSysID extends ManipulatorPivot {
    private MutVoltage appliedVoltage;

    private MutAngle pivotPosition;

    private MutAngularVelocity pivotVelocity;

    private SysIdRoutine routine;

    public ManipulatorPivotSysID() {
        super();

        appliedVoltage = Volts.mutable(0);

        pivotPosition = Radians.mutable(0);

        pivotVelocity = RadiansPerSecond.mutable(0);

        routine = new SysIdRoutine(
            ManipulatorPivotConstants.kSysIDConfig,
            new SysIdRoutine.Mechanism(
                pivotMotor::setVoltage, 
                (log) -> {
                    log.motor("armMotor")
                        .voltage(appliedVoltage.mut_replace(
                            pivotMotor.get() * RobotController.getBatteryVoltage(), Volts
                        ))
                        .angularPosition(pivotPosition.mut_replace(
                            getEncoderPosition(), Radians
                        ))
                        .angularVelocity(pivotVelocity.mut_replace(
                            getEncoderVelocity(), RadiansPerSecond
                        ));
                }, 
                this
            )
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
