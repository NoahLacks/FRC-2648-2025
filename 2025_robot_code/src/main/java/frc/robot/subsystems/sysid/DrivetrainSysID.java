package frc.robot.subsystems.sysid;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainSysID extends Drivetrain {
    public enum RoutineType {
        kDriving,
        kTurning
    }
    
    private MutVoltage drivingAppliedVoltage;
    private MutVoltage turningAppliedVoltage;

    private MutDistance drivingPosition;
    
    private MutAngle turningPosition;

    private MutLinearVelocity drivingVelocity;

    private SysIdRoutine drivingRoutine;
    private SysIdRoutine turningRoutine;
    
    public DrivetrainSysID() {
        super();

        drivingAppliedVoltage = Volts.mutable(0);
        turningAppliedVoltage = Volts.mutable(0);

        drivingPosition = Meters.mutable(0);

        turningPosition = Radians.mutable(0);

        drivingVelocity = MetersPerSecond.mutable(0);

        // TODO Does there need to be a mutable for each metric for each motor, or are reusing the three OK
        drivingRoutine = new SysIdRoutine(
            DrivetrainConstants.kSysIDDrivingConfig, 
            new SysIdRoutine.Mechanism(
                this::setDriveVoltageAll, 
                (log) -> {
                    log.motor("frontLeftDrive")
                        .voltage(drivingAppliedVoltage.mut_replace(m_frontLeft.getVoltageDrive(), Volts))
                        .linearPosition(drivingPosition.mut_replace(m_frontLeft.getPosition().distanceMeters, Meters))
                        .linearVelocity(drivingVelocity.mut_replace(m_frontLeft.getState().speedMetersPerSecond, MetersPerSecond));
                    log.motor("rearLeftDrive")
                        .voltage(drivingAppliedVoltage.mut_replace(m_rearLeft.getVoltageDrive(), Volts))
                        .linearPosition(drivingPosition.mut_replace(m_rearLeft.getPosition().distanceMeters, Meters))
                        .linearVelocity(drivingVelocity.mut_replace(m_rearLeft.getState().speedMetersPerSecond, MetersPerSecond));
                    log.motor("frontRightDrive")
                        .voltage(drivingAppliedVoltage.mut_replace(m_frontRight.getVoltageDrive(), Volts))
                        .linearPosition(drivingPosition.mut_replace(m_frontRight.getPosition().distanceMeters, Meters))
                        .linearVelocity(drivingVelocity.mut_replace(m_frontRight.getState().speedMetersPerSecond, MetersPerSecond));
                    log.motor("rearRightDrive")
                        .voltage(drivingAppliedVoltage.mut_replace(m_rearRight.getVoltageDrive(), Volts))
                        .linearPosition(drivingPosition.mut_replace(m_rearRight.getPosition().distanceMeters, Meters))
                        .linearVelocity(drivingVelocity.mut_replace(m_rearRight.getState().speedMetersPerSecond, MetersPerSecond));
                }, 
                this
            )
        );

        // TODO Does there need to be a mutable for each metric for each motor, or are reusing the two OK
        // TODO Angular velocity is not tracked because it's not easy to get by default, is this OK
        turningRoutine = new SysIdRoutine(
            DrivetrainConstants.kSysIDTurningConfig, 
            new SysIdRoutine.Mechanism(
                this::setTurnVoltageAll, 
                (log) -> {
                    log.motor("frontLeftTurn")
                        .voltage(turningAppliedVoltage.mut_replace(m_frontLeft.getVoltageTurn(), Volts))
                        .angularPosition(turningPosition.mut_replace(m_frontLeft.getState().angle.getRadians(), Radians));
                    log.motor("rearLeftTurn")
                        .voltage(turningAppliedVoltage.mut_replace(m_rearLeft.getVoltageTurn(), Volts))
                        .angularPosition(turningPosition.mut_replace(m_rearLeft.getState().angle.getRadians(), Radians));
                    log.motor("frontRightTurn")
                        .voltage(turningAppliedVoltage.mut_replace(m_frontRight.getVoltageTurn(), Volts))
                        .angularPosition(turningPosition.mut_replace(m_frontRight.getState().angle.getRadians(), Radians));
                    log.motor("rearRightTurn")
                        .voltage(turningAppliedVoltage.mut_replace(m_rearRight.getVoltageTurn(), Volts))
                        .angularPosition(turningPosition.mut_replace(m_rearRight.getState().angle.getRadians(), Radians));
                },
                this
            )
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, RoutineType type) {
        switch(type) {
            case kDriving:
                return makeDriveStraight().andThen(drivingRoutine.quasistatic(direction));
            case kTurning:
            default:
                return turningRoutine.quasistatic(direction);
        }
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, RoutineType type) {
        switch(type) {
            case kDriving:
                return makeDriveStraight().andThen(drivingRoutine.dynamic(direction));
            case kTurning:
            default:
                return turningRoutine.dynamic(direction);
        }
    }

    public void setDriveVoltageAll(Voltage voltage) {
        m_frontLeft.setVoltageDrive(voltage.magnitude());
        m_rearLeft.setVoltageDrive(voltage.magnitude());
        m_frontRight.setVoltageDrive(voltage.magnitude());
        m_rearRight.setVoltageDrive(voltage.magnitude());
    }

    public void setTurnVoltageAll(Voltage voltage) {
        m_frontLeft.setVoltageTurn(voltage.magnitude());
        m_rearLeft.setVoltageTurn(voltage.magnitude());
        m_frontRight.setVoltageTurn(voltage.magnitude());
        m_rearRight.setVoltageTurn(voltage.magnitude());
    }

    // TODO Does this nonsense actually work
    private Command makeDriveStraight() {
        return runOnce(() -> {
            SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

            this.setModuleStates(new SwerveModuleState[] {
                state,
                state,
                state,
                state
            });
        });
    }
}
