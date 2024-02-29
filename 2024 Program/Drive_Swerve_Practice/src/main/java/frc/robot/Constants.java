/** Copyright (c) FIRST and other WPILib contributors.
* Open Source Software; you can modify and/or share it under the terms of
* the WPILib BSD license file in the root directory of this project.
*/

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TractorToolbox.TractorParts.PIDGains;
//import frc.robot.commands.Limelight.LLAlignCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class ModuleConstants {

		// Current limits for the wheels
		public static final int kTurnMotorCurrentLimit = 25;
		public static final int kDriveMotorCurrentLimit = 30;

		// Constants set for the _SDS MK4i_
		public static final double kdriveGearRatio = 1d / 6.75;
		public static final double kturnGearRatio = 1d / (150d / 7d);

		public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;

		// The max speed the modules are capable of
		public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(14.5);

		public static final double ksVolts = .1;
		public static final double kDriveFeedForward = .2;

		// TODO: Retune feedforward values for turning
		public static final double kvTurning = .43205;
		public static final double ksTurning = .17161; // Tuned February 2, 2023

		// NEO drive motor CAN ID's
		public static final int kFrontLeftDriveMotorPort = 11;
		public static final int kRearLeftDriveMotorPort = 12;
		public static final int kRearRightDriveMotorPort = 13;
		public static final int kFrontRightDriveMotorPort = 14;

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 21;
		public static final int kRearLeftTurningMotorPort = 22;
		public static final int kRearRightTurningMotorPort = 23;
		public static final int kFrontRightTurningMotorPort = 24;

		// CANcoder CAN ID's
		public static final int kFrontLeftTurningEncoderPort = 1;
		public static final int kRearLeftTurningEncoderPort = 2;
		public static final int kRearRightTurningEncoderPort = 3;
		public static final int kFrontRightTurningEncoderPort = 4;

		// Offset angle for absolute encoders (find this using CTRE client)
		public static final double kFrontLeftAngleZero = .719727;//Units.rotationsToRadians((.283203)); 
		public static final double kRearLeftAngleZero = .277344;//Units.rotationsToRadians(-.278320);
		public static final double kRearRightAngleZero = .641846;//Units.rotationsToRadians(.358887);
		public static final double kFrontRightAngleZero = .034180;//Units.rotationsToRadians(-.036377);

		public static final PIDGains kModuleDriveGains = new PIDGains(.1, 0, 0);
		public static final PIDGains kModuleTurningGains = new PIDGains(1.5, 0, 0.0016);

	}

	public static class DriveConstants {

		public static final double kMaxSneakMetersPerSecond = 1.0;
		public static final double kMaxSpeedMetersPerSecond = 4.5;
		public static final double kMaxTurboMetersPerSecond = 8.0;

		// this sets turning speed (keep this low)
		public static final double kMaxRPM = 10;

		public static final int kPigeonPort = 20;

		public static final double kBumperToBumperWidth = Units.inchesToMeters(35.54);

		public static final double kTrackWidth = Units.inchesToMeters(23.75); // in meters!
		public static final double kWheelBase = Units.inchesToMeters(23.75); // in meters!

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR

		public static final boolean kGyroReversed = false;
		public static final boolean kFeildCentric = true;
		public static final boolean kGyroTurning = false;

		public static final PIDGains kGyroTurningGains = new PIDGains(.025, 0, 0);
		public static final double kMaxTurningVelocityDegrees = 20;
		public static final double kMaxTurningAcceleratonDegrees = 10;
		public static final double kGyroTurnTolerance = 2;

	}

	/**
	 * The constants pertaining to Autonoumus
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			public static final PIDGains kPPDriveGains = new PIDGains(8.5, 0, 0);
			public static final PIDGains kPPTurnGains = new PIDGains(3.5, 0, 0);

			public static final double kPPMaxVelocity = 4.5;
			public static final double kPPMaxAcceleration = 3.0;

			public static final HashMap<String, Command> kPPEventMap = new HashMap<>() {
				{
					//put("TargetTape", new LLAlignCommand(false));
					//put("TargetTag", new LLAlignCommand(true));
				}
			};
		}

		public static final double kScoreSequenceDropTime = 3; // in seconds

		public static final PIDGains kTurnCommandGains = new PIDGains(.004, 0, 0);
		public static final double kTurnCommandMaxVelocity = 1;
		public static final double kTurnCommandMaxAcceleration = 1;
		public static final double kTurnCommandToleranceDeg = 0.5;
		public static final double kTurnCommandRateToleranceDegPerS = 0;

		public static final double kBalnaceCommandDeadbandDeg = -2.5;
		public static final PIDGains kBalanceCommandGains = new PIDGains(.019, 0, 0);
		public static final double kMaxBalancingVelocity = 1000;
		public static final double kMaxBalancingAcceleration = 5000;
	}

	/**
	 * The constants pertaining to the drive station
	 */
	public static class OperatorConstants {
		public static final int kDriveJoystickPort = 0;
		public static final int kTurnJoystickPort = 1;
		public static final int kOperatorControllerPort = 2;
		public static final int kProgrammerControllerPort = 3;

		public static final double KDeadBand = .125;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;
	}

	public static class LimelightConstants {

		// declare ID's of pipelines here
		public static final int kCubePipeline = 0;
		public static final int kReflectivePipeline = 1;
		public static final int kApriltagPipeline = 2;

		// PID values for limelight
		public static final PIDGains kLLTargetGains = new PIDGains(0.008, 0, 0);

		public static final PIDGains kLLPuppyTurnGains = new PIDGains(0.02, 0, 0); //.008
		public static final PIDGains kLLPuppyDriveGains = new PIDGains(0.008, 0, 0);
		public static final double kPuppyTurnMotionSmoothing = 0.3;
		public static final double kPuppyDriveMotionSmoothing = 0.4;

		public static final PIDGains kLLAlignStrafeGains = new PIDGains(.04, 0.0015, 0.001);
		public static final PIDGains kLLAlignDriveGains = new PIDGains(.025, 0.0015, 0.0005);
		public static final double kAlignDriveMotionSmoothing = 0;
		public static final double kAlignStrafeMotionSmoothing = 0;
	}

	public static final String kRioCANBusName = "rio";

	public static final String kDriveCANBusName = "canivore";

	public static final int ARM_MOTOR_1 = 17;
	public static final int ARM_MOTOR_2 = 10;
	public static final int SLED_MOTOR_ID = 18;

	public static final int CLAW_SOLENOID_ID1 = 1;
	public static final int CLAW_SOLENOID_ID2 = 0;
	public static final int REVPH_ID = 5;
	
	public static final int ARM_ENCODER = 7;
	public static final int SLED_ENCODER_ID1 = 8;
	public static final int SLED_ENCODER_ID2 = 9;
  
	public static final int EXTEND_LIMITSWITCH_ID = 1;
	public static final int RETRACT_LIMITSWITCH_ID = 0;

	public static final double ABOFFSET = 0.03;
	public static final double SLED_MOTOR_DRIVE_KP = 0.0005;
	public static final double AB_MOTOR_DRIVE_KP = 7;

	public static final int XBOXCONTROLLER_ID = 2;

	public static final int BUTTON_12_ID = 12;
	public static final int BUTTON_10_ID = 10;
	public static final int BUTTON_9_ID = 9;
	public static final int BUTTON_8_ID = 8;
	public static final int BUTTON_7_ID = 7;
	public static final int BUTTON_2_ID = 2;
	public static final int BUTTON_1_ID = 1;

	public static final double MAX_ARM_ENCODER = 0.88;
	public static final double MIN_ARM_ENCODER = 0.65;

	public static final double SLED_HOME = 300;
	public static final double SLED_LOADING = 300;
	public static final double SLED_LOW = 1680;
	public static final double SLED_MEDIUM = 5775;
	public static final double SLED_HIGH = 13000;
}
