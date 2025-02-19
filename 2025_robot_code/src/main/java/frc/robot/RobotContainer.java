// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.ManipulatorPivotConstants;
import frc.robot.constants.ClimberPivotConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.ManipulatorPivot;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ClimberPivot;
import frc.robot.subsystems.ClimberRollers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.sysid.ElevatorSysID;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {
	private ClimberPivot climberPivot;

	private ClimberRollers climberRollers;

	private Drivetrain drivetrain;

	private Elevator elevator;
	//private ElevatorSysID elevator;

	private Manipulator manipulator;

	private ManipulatorPivot manipulatorPivot;

	private CommandXboxController driver;
	private CommandXboxController operator;

	private SendableChooser<Command> autoChooser;
	private Vision vision;

	public RobotContainer() {
		climberPivot = new ClimberPivot();

		climberRollers = new ClimberRollers();

		//vision = new Vision(drivetrain::getGyroValue);
		drivetrain = new Drivetrain();

		elevator = new Elevator();
		//elevator = new ElevatorSysID();

		manipulator = new Manipulator();

		manipulatorPivot = new ManipulatorPivot();

		driver = new CommandXboxController(OIConstants.kDriverControllerPort);
		operator = new CommandXboxController(OIConstants.kOperatorControllerPort);

		autoChooser = AutoBuilder.buildAutoChooser();

		configureButtonBindings();
		//elevatorSysIDBindings();

		configureNamedCommands();

		configureShuffleboard();
	}

	/*private void elevatorSysIDBindings() {
		elevator.setDefaultCommand(elevator.maintainPosition());

		operator.a().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
		operator.b().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
		operator.x().whileTrue(elevator.sysIdDynamic(Direction.kForward));
		operator.y().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
	}*/

	private void configureButtonBindings() {
		//Default commands
		climberPivot.setDefaultCommand(
			climberPivot.runPivot(0)
		);

		climberRollers.setDefaultCommand(
			climberRollers.runRoller(0)
		);

		drivetrain.setDefaultCommand(
			drivetrain.drive(
				driver::getLeftY,
				driver::getLeftX,
				driver::getRightX,
				() -> true
			)
		);

		elevator.setDefaultCommand(
			elevator.goToSetpoint(
				() -> 0
			)
		);

		
		manipulatorPivot.setDefaultCommand(
			manipulatorPivot.runManualPivot(
				() -> 0
			)
		);
		

		manipulator.setDefaultCommand(
			manipulator.runUntilCollected(
				() -> 0
			)
		);

		//Driver inputs
		/*
		driver.start().whileTrue(
			drivetrain.setXCommand()
		);

		driver.rightTrigger().whileTrue(
			manipulator.runManipulator(() -> 1, true)
		);

		driver.leftTrigger().whileTrue( 
			manipulator.runUntilCollected(() -> 0.75)
		);

		driver.start().and(driver.back()).onTrue(
			startingConfig()
		);
		*/
		driver.povDown().whileTrue(climberPivot.runPivot(-0.5));
		driver.povUp().whileTrue(climberPivot.runPivot(0.5));

		driver.povLeft().whileTrue(climberRollers.runRoller(0.5));
		driver.povRight().whileTrue(climberRollers.runRoller(-0.5));
		
		
		operator.povUp().whileTrue(
			elevator.goToSetpoint(() -> 50)
		);

		operator.povDown().whileTrue(
			elevator.goToSetpoint(() -> 0)
		);

		/*
		operator.a().whileTrue(elevator.runManualElevator(() -> 0.2));

		operator.b().whileTrue(elevator.runManualElevator(() -> -0.2));

 
		//Operator inputs
		operator.povUp().onTrue(
			moveManipulator(
				ElevatorConstants.kL4Position, 
				ManipulatorPivotConstants.kL4Position
			)
		);

		operator.povRight().onTrue(
			moveManipulator(
				ElevatorConstants.kL3Position, 
				ManipulatorPivotConstants.kL3Position
			)
		);

		operator.povLeft().onTrue(
			moveManipulator(
				ElevatorConstants.kL2Position, 
				ManipulatorPivotConstants.kL2Position
			)
		);

		operator.povDown().onTrue(
			moveManipulator(
				ElevatorConstants.kL1Position, 
				ManipulatorPivotConstants.kL1Position
			)
		);

		operator.a().onTrue(
			coralIntakeRoutine()
		);

		operator.x().onTrue(
			algaeIntakeRoutine(true)
		);

		operator.b().onTrue(
			algaeIntakeRoutine(false)
		);
		*/

	}

	private void configureNamedCommands() {
		NamedCommands.registerCommand("Drivetrain Set X", drivetrain.setXCommand());
	}

	//creates tabs and transforms them on the shuffleboard
    private void configureShuffleboard() {
		ShuffleboardTab autoTab = Shuffleboard.getTab(OIConstants.kAutoTab);
    	ShuffleboardTab sensorTab = Shuffleboard.getTab(OIConstants.kSensorsTab);

		Shuffleboard.selectTab(OIConstants.kAutoTab);

		autoTab.add("Auto Selection", autoChooser)
			.withSize(2, 1)
			.withPosition(0, 0)
			.withWidget(BuiltInWidgets.kComboBoxChooser);

		sensorTab.addDouble("Elevator Position", elevator::getEncoderPosition)
			.withSize(2, 1)
			.withPosition(0, 0)
			.withWidget(BuiltInWidgets.kGraph);

		sensorTab.addDouble("Manipulator Position", manipulatorPivot::getEncoderPosition)
			.withSize(2, 1)
			.withPosition(2, 0)
			.withWidget(BuiltInWidgets.kTextView);

		sensorTab.addDouble("Climber Pivot Position", climberPivot::getEncoderPosition)
			.withSize(2, 1)
			.withPosition(2, 1)
			.withWidget(BuiltInWidgets.kTextView);
			
		sensorTab.addDouble("gyro angle", drivetrain::getGyroValue)
			.withSize(2, 1)
			.withPosition(0, 1)
			.withWidget(BuiltInWidgets.kTextView);

		sensorTab.addBoolean("Coral Sensor", manipulator::getCoralBeamBreak)
			.withSize(1, 1)
			.withPosition(4, 0)
			.withWidget(BuiltInWidgets.kBooleanBox);

		sensorTab.addBoolean("bottom limit switch", elevator::getBottomLimitSwitch)
			.withSize(1, 1)
			.withPosition(4, 1)
			.withWidget(BuiltInWidgets.kBooleanBox);

		sensorTab.addDouble("ElevMotor1", elevator::getMotor1)
			.withWidget(BuiltInWidgets.kGraph);

		sensorTab.addDouble("ElevMotor2", elevator::getMotor2)
			.withWidget(BuiltInWidgets.kGraph);

		sensorTab.addDouble("PID output", elevator::currentPIDOut)
		.withWidget(BuiltInWidgets.kGraph);
 	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	/**
	 * Moves the elevator and arm to the coral intake position, then runs the manipulator until collected
	 * @return Moves the elevator and arm, then intakes coral
	 */
	@SuppressWarnings("unused")
	private Command coralIntakeRoutine() {
		return moveManipulator(
			ElevatorConstants.kCoralIntakePosition, 
			ManipulatorPivotConstants.kCoralIntakePosition
		)
			.andThen(manipulator.runUntilCollected(() -> .5));
		}

	/**
	 * Moves the elevator and arm to the constant setpoints and runs the manipulator until collected
	 * 
	 * @param l2 Is the algae on L2? (True = L2, False = L3)
	 * @return Moves the elevator and arm then intakes algae
	 */
	@SuppressWarnings("unused")
	private Command algaeIntakeRoutine(boolean l2) {
		return moveManipulator(
			l2 ? ElevatorConstants.kL2AlgaePosition : ElevatorConstants.kL3AlgaePosition, 
			l2 ? ManipulatorPivotConstants.kL2AlgaePosition : ManipulatorPivotConstants.kL3AlgaePosition
		)
			.andThen(manipulator.runUntilCollected(() -> 1));
	}

	/**
	 * Moves the elevator and arm in different order based on target positions
	 * 
	 * @param elevatorPosition The target position of the elevator
	 * @param armPosition The target rotation of the arm
	 * @return Moves the elevator and arm to the setpoints using the most efficient path
	 */
	private Command moveManipulator(double elevatorPosition, double armPosition) {
		// If the elevator current and target positions are above the brace, or the arm current and target position is in
		// front of the brace, move together
		if ((elevator.isMotionSafe() && elevator.isMotionSafe(elevatorPosition)) || (manipulatorPivot.isMotionSafe() && manipulatorPivot.isMotionSafe(armPosition))) { 
			return moveManipulatorUtil(elevatorPosition, armPosition, false, false);
		// If the target position is behind the brace, and the arm is not behind the brace, move the arm to a safe position first,
		// then the elevator, then the arm again
		} else if (!manipulatorPivot.isMotionSafe(armPosition) && !manipulatorPivot.isMotionSafe()) { 
			return moveManipulatorUtil(elevatorPosition, ManipulatorPivotConstants.kPivotSafeStowPosition, false, true)
				.andThen(manipulatorPivot.goToSetpoint(armPosition));
		// If the target position is behind the brace, and the arm is behind the brace, move the elevator first, then the arm
		} else if (!manipulatorPivot.isMotionSafe(armPosition) && manipulatorPivot.isMotionSafe()) {
			return moveManipulatorUtil(elevatorPosition, armPosition, true, true);
		// If the arm is behind the brace, move the arm first, then the elevator
		} else if (!manipulatorPivot.isMotionSafe()) {
			return moveManipulatorUtil(elevatorPosition, armPosition, false, true);
		// Catch all command that's safe regardless of arm and elevator positions
		} else { 
			return moveManipulatorUtil(elevatorPosition, ManipulatorPivotConstants.kPivotSafeStowPosition, false, true)
				.andThen(manipulatorPivot.goToSetpoint(armPosition));
		}
	}

	/**
	 * Moves the elevator and arm in customizeable ways
	 * 
	 * @param elevatorPosition The target elevator position
	 * @param armPosition The target arm position
	 * @param elevatorFirst Does the elevator move first? (True = Elevator first, False = Arm first)
	 * @param sequential Does the elevator and arm move separately? (True = .andThen, False = .alongWith)
	 * @return Moves the elevator and arm to the setpoints
	 */
	private Command moveManipulatorUtil(double elevatorPosition, double armPosition, boolean elevatorFirst, boolean sequential) {
		if (elevatorPosition <= ElevatorConstants.kBracePosition || elevatorPosition == 0) {
			armPosition = MathUtil.clamp(
				armPosition, 
				0,
				ManipulatorPivotConstants.kRotationLimit
			);
		}
		
		return Commands.either(
			Commands.either(
				elevator.goToSetpoint(() -> elevatorPosition).andThen(manipulatorPivot.goToSetpoint(armPosition)),
				elevator.goToSetpoint(() -> elevatorPosition).alongWith(manipulatorPivot.goToSetpoint(armPosition)),
				() -> sequential
			), 
			Commands.either(
				manipulatorPivot.goToSetpoint(armPosition).andThen(elevator.goToSetpoint(() -> elevatorPosition)), 
				manipulatorPivot.goToSetpoint(armPosition).alongWith(elevator.goToSetpoint(() -> elevatorPosition)), 
				() -> sequential
			), 
			() -> elevatorFirst
		);
	}

	@SuppressWarnings("unused")
	private Command manipulatorSafeTravel(double elevatorPosition, double armPosition, boolean isL4){
		if(!isL4){
			return Commands.sequence(
				manipulatorPivot.goToSetpoint(ManipulatorPivotConstants.kPivotSafeStowPosition),
				elevator.goToSetpoint(() -> elevatorPosition),
				manipulatorPivot.goToSetpoint(armPosition));
			
		}else{
			return Commands.sequence(
				manipulatorPivot.goToSetpoint(ManipulatorPivotConstants.kPivotSafeStowPosition),
				elevator.goToSetpoint(() -> elevatorPosition).until(() -> elevator.getEncoderPosition() > ElevatorConstants.kL4TransitionPosition),
				Commands.parallel( manipulatorPivot.goToSetpoint(armPosition)), elevator.goToSetpoint(() -> elevatorPosition));
		}
		
	}

	/**
	 * Moves the arm and elevator in a safe way.
	 * 
	 * @param elevatorPosition The target position of the elevator
	 * @param armPosition The target rotation of the arm
	 * @return Moves the elevator and arm to the setpoints
	 */
	@SuppressWarnings("unused")
	private Command safeMoveManipulator(double elevatorPosition, double armPosition) {
		return moveManipulatorUtil(elevatorPosition, ManipulatorPivotConstants.kPivotSafeStowPosition, false, true)
			.andThen(manipulatorPivot.goToSetpoint(armPosition));
	}

	@SuppressWarnings("unused")
	private Command startingConfig() {
		return moveManipulatorUtil(0, 0, false, true)
			.alongWith(climberPivot.climb(ClimberPivotConstants.kClimberStartingPosition, .1));
	}
}