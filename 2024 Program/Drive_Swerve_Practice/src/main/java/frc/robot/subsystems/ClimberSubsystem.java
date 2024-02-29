// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final MotorController climbL = new WPI_VictorSPX(15);
  private final MotorController climbR = new WPI_VictorSPX(16); 

  public ClimberSubsystem() {

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public void setMotor(double speed) {
    climbR.set(speed);
    climbL.set(speed);
  }
}