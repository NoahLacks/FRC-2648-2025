// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.constants.ModuleConstants;

public class MAXSwerveModule {
    private final TalonFX m_drive;
    private final SparkMax m_turningSpark;

    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_turningClosedLoopController;

    private final VelocityVoltage driveVelocityRequest;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drive = new TalonFX(drivingCANId);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        driveVelocityRequest = new VelocityVoltage(0).withSlot(0);

        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        m_drive.getConfigurator().apply(ModuleConstants.kDriveCurrentLimitConfig);
        m_drive.getConfigurator().apply(ModuleConstants.kDriveFeedConfig);
        m_drive.getConfigurator().apply(ModuleConstants.kDriveMotorConfig);
        m_drive.getConfigurator().apply(ModuleConstants.kDriveSlot0Config);

        m_turningSpark.configure(ModuleConstants.turningConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drive.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drive.getVelocity().getValueAsDouble(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(m_drive.getPosition().getValueAsDouble(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS towards their respective setpoints.
        m_drive.setControl(
        driveVelocityRequest.withVelocity(
            correctedDesiredState.speedMetersPerSecond
        ).withFeedForward(
            correctedDesiredState.speedMetersPerSecond
        )
        );

        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }

    public void setVoltageDrive(double voltage){
        m_drive.setVoltage(voltage);
    }

    public void setVoltageTurn(double voltage) {
        m_turningSpark.setVoltage(voltage);
    }

    public double getVoltageDrive() {
        return m_drive.get() * RobotController.getBatteryVoltage();
    }

    public double getVoltageTurn() {
        return m_turningSpark.get() * RobotController.getBatteryVoltage();
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drive.setPosition(0);
    }
}
