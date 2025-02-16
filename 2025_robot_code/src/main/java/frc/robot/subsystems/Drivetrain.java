// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.OIConstants;

public class Drivetrain extends SubsystemBase {
    // Create MAXSwerveModules
    protected MAXSwerveModule m_frontLeft;
    protected MAXSwerveModule m_frontRight;
    protected MAXSwerveModule m_rearLeft;
    protected MAXSwerveModule m_rearRight;

    // The gyro sensor
    private AHRS ahrs;

    // Odometry class for tracking robot pose
    private SwerveDrivePoseEstimator m_estimator;

    /** Creates a new DriveSubsystem. */
    public Drivetrain() {
        m_frontLeft = new MAXSwerveModule(
            DrivetrainConstants.kFrontLeftDrivingCanId,
            DrivetrainConstants.kFrontLeftTurningCanId,
            DrivetrainConstants.kFrontLeftChassisAngularOffset
        );

        m_frontRight = new MAXSwerveModule(
            DrivetrainConstants.kFrontRightDrivingCanId,
            DrivetrainConstants.kFrontRightTurningCanId,
            DrivetrainConstants.kFrontRightChassisAngularOffset
        );

        m_rearLeft = new MAXSwerveModule(
            DrivetrainConstants.kRearLeftDrivingCanId,
            DrivetrainConstants.kRearLeftTurningCanId,
            DrivetrainConstants.kBackLeftChassisAngularOffset
        );

        m_rearRight = new MAXSwerveModule(
            DrivetrainConstants.kRearRightDrivingCanId,
            DrivetrainConstants.kRearRightTurningCanId,
            DrivetrainConstants.kBackRightChassisAngularOffset
        );

        ahrs = new AHRS(NavXComType.kMXP_SPI);

        m_estimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.kDriveKinematics,
            Rotation2d.fromDegrees(ahrs.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            new Pose2d()
        );

        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getCurrentChassisSpeeds, 
            this::driveWithChassisSpeeds, 
            AutoConstants.kPPDriveController, 
            AutoConstants.kRobotConfig,
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
            }, 
            this
        );
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_estimator.update(
            Rotation2d.fromDegrees(getGyroValue()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        });
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return DrivetrainConstants.kDriveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        );
    }

    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.2);
        SwerveModuleState[] newStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);

        setModuleStates(newStates);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_estimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_estimator.resetPose(
            pose
        );
    }

    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot,
            BooleanSupplier fieldRelative) {
        return run(() -> {
            drive(
                -MathUtil.applyDeadband(xSpeed.getAsDouble(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(ySpeed.getAsDouble(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rot.getAsDouble(), OIConstants.kDriveDeadband),
                fieldRelative.getAsBoolean()
            );
        });
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DrivetrainConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DrivetrainConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DrivetrainConstants.kMaxAngularSpeed;

        var swerveModuleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                        Rotation2d.fromDegrees(getGyroValue()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public Command setXCommand() {
        return run(() -> {
            setX();
        });
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        ahrs.reset();
    }

    public double getGyroValue() {
        return ahrs.getAngle() * (DrivetrainConstants.kGyroReversed ? -1 : 1);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(getGyroValue()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return ahrs.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp){
        m_estimator.addVisionMeasurement(pose, timestamp);
    }
}
