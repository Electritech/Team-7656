// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.TractorToolbox.JoystickUtils;
//import frc.robot.TractorToolbox.TractorParts.PathBuilder;
//import frc.robot.commands.ClawToggle;
//import frc.robot.commands.DriveOutAuto;
//import frc.robot.commands.LeftAuto2Piece;
//import frc.robot.commands.ManualModeToggle;
//import frc.robot.commands.MiddleBalance;
//import frc.robot.commands.TurnCommand;
//import frc.robot.commands.Autonomous.BalanceCommand;
//import frc.robot.commands.Limelight.LLAlignCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.arm;
//import frc.robot.subsystems.clawSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// The robot's subsystems and commands are defined here...
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
	//public static arm m_arm = new arm();
	//public static clawSubsystem m_claw = new clawSubsystem();
	//private Command m_driveOut = new DriveOutAuto();
	//private Command m_middleBalance = new MiddleBalance();
	//private Command m_leftAuto2Piece = new LeftAuto2Piece();


	//public final static PathBuilder autoBuilder = new PathBuilder();

	private final CommandJoystick driveJoystick = new CommandJoystick(
			OperatorConstants.kDriveJoystickPort);
	private final CommandJoystick turnJoystick = new CommandJoystick(
			OperatorConstants.kTurnJoystickPort);
	//private final CommandGenericHID operatorController = new CommandGenericHID(
			//OperatorConstants.kOperatorControllerPort);
	private final CommandXboxController programmerController = new CommandXboxController(
			OperatorConstants.kProgrammerControllerPort);

	private Joystick Leftjoy = new Joystick(OperatorConstants.kDriveJoystickPort);
	private Joystick Rightjoy = new Joystick(OperatorConstants.kTurnJoystickPort);
	private XboxController Controller1 = new XboxController(Constants.XBOXCONTROLLER_ID);
	private CommandXboxController ccontroller = new CommandXboxController(Constants.XBOXCONTROLLER_ID);
	public JoystickButton button12 = new JoystickButton(Leftjoy, Constants.BUTTON_12_ID);
	public JoystickButton button10 = new JoystickButton(Leftjoy, Constants.BUTTON_10_ID);
	public JoystickButton rightButton12 = new JoystickButton(Rightjoy, Constants.BUTTON_12_ID);
	public JoystickButton rightButton11 = new JoystickButton(Rightjoy, 11);
	public JoystickButton button9 = new JoystickButton(Leftjoy, Constants.BUTTON_9_ID);
	public JoystickButton button8 = new JoystickButton(Leftjoy, Constants.BUTTON_8_ID);
	public JoystickButton button7 = new JoystickButton(Leftjoy, Constants.BUTTON_7_ID);
	public JoystickButton lTrigger = new JoystickButton(Leftjoy, Constants.BUTTON_1_ID);
	public JoystickButton rTrigger = new JoystickButton(Rightjoy, Constants.BUTTON_1_ID);
	public JoystickButton button2 = new JoystickButton(Rightjoy, Constants.BUTTON_2_ID);
	Trigger xButton = ccontroller.x(); // Creates a new Trigger object for the `X` button on exampleCommandController
	Trigger yButton = ccontroller.y();
	boolean abutton = Controller1.getAButtonPressed();
	Trigger lbumper = ccontroller.leftBumper();
	Trigger rbumper = ccontroller.rightBumper();
	Trigger XboxButton9 = ccontroller.button(9);
	Trigger XboxButton10 = ccontroller.button(10);
	Trigger backButton = ccontroller.back();
	private SendableChooser<Command> autoChooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		// region Def Auto
		Shuffleboard.getTab("Driver").add(autoChooser);

		//autoBuilder.populatePathMap();

		//autoChooser.addOption("MC Hammers 1", autoBuilder.getPathCommand("MC Hammers 1"));
		//autoChooser.addOption("Triangle", autoBuilder.getPathCommand("PathPlanner Fun"));
		//autoChooser.addOption("DriveOut", new SequentialCommandGroup(new DriveOutAuto(), autoBuilder.getPathCommand("1 Ball High")));
		//autoChooser.addOption("Middle Balance", m_middleBalance);
		//autoChooser.addOption("2 Piece Left", new SequentialCommandGroup(new DriveOutAuto(), autoBuilder.getPathCommand("2 Piece Left"), m_leftAuto2Piece));
		//autoChooser.addOption("Path Planner Test", autoBuilder.getPathCommand("1 Ball High"));
		//autoChooser.addOption("2 Piece Left", new SequentialCommandGroup(m_driveOut, autoBuilder.getPathCommand("2 Piece Left")));
		// endregion
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {

		// region Targeting Commmands
		// driveJoystick.button(3).whileTrue(new LLAlignCommand(false));
		// driveJoystick.button(4).whileTrue(new LLAlignCommand(true));
		// driveJoystick.button(5).whileTrue(new BalanceCommand());
		//button12.onTrue(new BalanceCommand());
		// programmerController.a().whileTrue(new LLAlignCommand(false));
		// programmerController.x().whileTrue(new TurnCommand(180));
		// endregion

		// test balance
		// operatorController.button(12).whileTrue(new BalanceCommand());

		// region Drive Commands
		// driveJoystick.button(11).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		// driveJoystick.button(12).onTrue(driveSubsystem.toggleFieldCentric());

		// programmerController.button(8).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		// programmerController.button(6).onTrue(driveSubsystem.toggleFieldCentric());
		//lTrigger.onTrue(new InstantCommand(() -> driveSubsystem.resetEncoders()));
		rightButton11.onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		rightButton12.onTrue(driveSubsystem.toggleFieldCentric());
		//button12.whileTrue(new BalanceCommand());
		// Runs limemlightCommand while button 7 is pressed
		//backButton.onTrue(new ManualModeToggle());
		//rbumper.onTrue(new ClawToggle());

		driveJoystick.povUp().whileTrue(
				new RunCommand(() -> driveSubsystem.robotCentricDrive(-0.05, 0, 0), driveSubsystem));
		driveJoystick.povDown().whileTrue(
				new RunCommand(() -> driveSubsystem.robotCentricDrive(0.05, 0, 0), driveSubsystem));

		// Swerve Drive method is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				new RunCommand(
						() -> driveSubsystem.drive(
								JoystickUtils.processJoystickInput(driveJoystick.getY())
										+ JoystickUtils.processJoystickInput(programmerController.getLeftY()), // x axis
								JoystickUtils.processJoystickInput(driveJoystick.getX())
										+ JoystickUtils.processJoystickInput(programmerController.getLeftX()), // y axis
								JoystickUtils.processJoystickInput(turnJoystick.getX())
										+ JoystickUtils.processJoystickInput(programmerController.getRightX()), // rot axis

								driveJoystick.getHID().getRawButton(1), // turbo boolean
								driveJoystick.getHID().getRawButton(2)), // sneak boolean
						driveSubsystem));
		// endregion
	}

	public double GetXboxStickLeft() {
		return Controller1.getLeftY();
	}

	public double GetXboxStickRight() {
		return Controller1.getRightY();
	}

	public double getJoyStickTriggerL() {
		return Controller1.getLeftTriggerAxis();
	}

	public double getJoyStickTriggerR() {
		return Controller1.getRightTriggerAxis();
	}

	public boolean getdpad() {
		return Controller1.getPOV() >= 0;
	}

	public boolean getXboxButton9() {
		return XboxButton9.getAsBoolean();
	}

	public boolean getXboxButton10() {
		return XboxButton10.getAsBoolean();
	}

	public double isArmButtonPressed() {
		if (Controller1.getStartButton())
			// home pos
			return 0.87;
		if (Controller1.getYButton())
			// High pos
			return 0.66;
		if (Controller1.getBButton())
			// Medium pos
			return 0.70;
		if (Controller1.getAButton())
			// Low pos
			return 0.80;
		if (Controller1.getXButton()) {
			// Loading pos
			// return 0.84;
			return 0.71;
		}
		return Math.PI;
	}

	public double isArmButtonPressedBC() {
		if (Controller1.getStartButton())
			// Home
			return Constants.SLED_HOME;
		if (Controller1.getYButton())
			// High
			return Constants.SLED_HIGH;
		if (Controller1.getBButton())
			// Medium
			return Constants.SLED_MEDIUM;
		if (Controller1.getAButton())
			// Low
			return Constants.SLED_LOW;
		if (Controller1.getXButton()) {
			// Loading
			return Constants.SLED_LOADING;
		}
		return Math.PI;
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		driveSubsystem.setHeading(180);
		Timer.delay(0.05);
		// the command to be run in autonomous
		return autoChooser.getSelected();
	}
}