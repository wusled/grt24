// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;


public class Intake extends SubsystemBase {
  private final WPI_VictorSPX alis = new WPI_VictorSPX(57);

  /** Creates a new Intake. */
  public Intake() {}

    public void setIntakeVolts(double voltage){
      alis.setVoltage(voltage);
  }
 
  public Command setIntakeVoltsCommand(DoubleSupplier voltage){
    return runOnce(() -> setIntakeVolts(voltage.getAsDouble()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
