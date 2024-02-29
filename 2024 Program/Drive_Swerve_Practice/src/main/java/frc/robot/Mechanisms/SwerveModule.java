// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;


import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.BaseStatusSignal;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.TractorToolbox.TractorParts.PIDGains;

public class SwerveModule {
	/** Creates a new SwerveModule. */

	private final CANSparkMax driveMotor;
	private final CANSparkMax turnMotor;

	private final CoreCANcoder absoluteEncoder;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnEncoder;

	private final SparkPIDController drivePID;
	private final SparkPIDController turnPID;
	// private final ProfiledPIDController m_turningPIDController;

	public final double angleZero;
	private final String moduleName;

	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);
	
	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPIDGains,
			PIDGains drivePIDGains) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		// Initialize the motors
		driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		turnMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		driveMotor.setInverted(true);
		turnMotor.setInverted(true);

		//turnMotor.restoreFactoryDefaults();
		//driveMotor.restoreFactoryDefaults();
		
		//turnMotor.setInverted(true);
		//driveMotor.setInverted(true);

		// Initalize CANcoder
		absoluteEncoder = new CoreCANcoder(absoluteEncoderPort);
		Timer.delay(1);
		CANcoderConfiguration configs = new CANcoderConfiguration();
		//absoluteEncoder.configFactoryDefault();
		//absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		// sat absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		//absoluteEncoder.configMagnetOffset(-1 * angleZero);
		//absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);
		BaseStatusSignal.setUpdateFrequencyForAll(100, absoluteEncoder.getPosition(), absoluteEncoder.getFault_Undervoltage(), absoluteEncoder.getSupplyVoltage());
		configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
 		configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		//configs.MagnetSensor.MagnetOffset = angleZero;
		absoluteEncoder.getConfigurator().apply(configs);		
		absoluteEncoder.clearStickyFaults();

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(ModuleConstants.kdriveGearRatio * ModuleConstants.kwheelCircumference); // meters
		driveEncoder.setVelocityConversionFactor(
				ModuleConstants.kdriveGearRatio
						* ModuleConstants.kwheelCircumference
						* (1d / 60d)); // meters per second

		turnEncoder = turnMotor.getEncoder();
		turnEncoder.setPositionConversionFactor((2 * Math.PI) * ModuleConstants.kturnGearRatio);
		turnEncoder.setVelocityConversionFactor((2 * Math.PI) * ModuleConstants.kturnGearRatio * (1d / 60d));
		// sat turnEncoder.setPosition(Units.degreesToRadians(absoluteEncoder.getAbsolutePosition() - angleZero));
		var tEposition = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
	    turnEncoder.setPosition(tEposition - angleZero);
		// Initialize PID's
		drivePID = driveMotor.getPIDController();
		drivePID.setP(drivePIDGains.kP);
		drivePID.setI(drivePIDGains.kI);
		drivePID.setD(drivePIDGains.kD);

		turnPID = turnMotor.getPIDController();
		turnPID.setP(angularPIDGains.kP);
		turnPID.setI(angularPIDGains.kI);
		turnPID.setD(angularPIDGains.kD);

		// m_turningPIDController = new ProfiledPIDController(
		// angularPID.kP,
		// angularPID.kI,
		// angularPID.kD,
		// new TrapezoidProfile.Constraints( // radians/s?
		// 2 * Math.PI * 600, // theoretical is 5676 RPM -> 94*2pi
		// 2 * Math.PI * 1200));

		this.drivePID.setFF(ModuleConstants.kDriveFeedForward);

		this.drivePID.setFeedbackDevice(driveMotor.getEncoder());

		this.drivePID.setOutputRange(-1, 1);

		// Configure current limits for motors
		driveMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setSmartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);
		driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);

		turnPID.setPositionPIDWrappingEnabled(true);
		turnPID.setPositionPIDWrappingMinInput(-Math.PI);
		turnPID.setPositionPIDWrappingMaxInput(Math.PI);
		

		SmartDashboard.putNumber(this.moduleName + " Offset", angleZero);
		//SmartDashboard.putString(this.moduleName + " Abs. Status", absoluteEncoder.getLastError().toString());
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		//return absoluteEncoder.getAbsolutePosition().getValue();
		return turnEncoder.getPosition();
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {
		//s double moduleAngleRadians = Math.toRadians(eCoder.getValue() * 360);
		//double moduleAngleRadians = Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValue());
		double moduleAngleRadians = turnEncoder.getPosition();
		double distanceMeters = driveEncoder.getPosition();
		return new SwerveModulePosition(distanceMeters, new Rotation2d(moduleAngleRadians));
	}

	public void setDesiredState(SwerveModuleState desiredState) {		
		//double moduleAngleRadians = Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValue());
		double moduleAngleRadians = turnEncoder.getPosition();
		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(moduleAngleRadians));

		// final var angularPIDOutput =
		// m_turningPIDController.calculate(moduleAngleRadians,
		// optimizedState.angle.getRadians());

		// final var angularFFOutput =
		// turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

		// final var turnOutput = angularPIDOutput + angularFFOutput;

		// turnMotor.setVoltage(turnOutput);

		drivePID.setReference(
				optimizedState.speedMetersPerSecond,
				ControlType.kVelocity);

		turnPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);

		SmartDashboard.putNumber(this.moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		//SmartDashboard.putNumber(this.moduleName + " Turn Motor Output", turnMotor.getAppliedOutput());
		// SmartDashboard.putNumber(this.moduleName + " Turn Output", turnOutput);

	}

	public void resetEncoders() {
		turnEncoder.setPosition(0);
		//driveEncoder.setPosition(0);
		//absoluteEncoder.setPosition(0);
		//turnEncoder.setPositionConversionFactor((2 * Math.PI) * ModuleConstants.kturnGearRatio);
		//turnEncoder.setVelocityConversionFactor((2 * Math.PI) * ModuleConstants.kturnGearRatio * (1d / 60d));
		// sat turnEncoder.setPosition(Units.degreesToRadians(absoluteEncoder.getAbsolutePosition() - angleZero));
		//var tEposition = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
	    //turnEncoder.setPosition(tEposition - angleZero);
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turnMotor.stopMotor();
	}

}
