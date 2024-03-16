// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Shooter extends SubsystemBase {
  private final WPI_VictorSPX sol_atis = new WPI_VictorSPX(13);
  private final WPI_VictorSPX sag_atis = new WPI_VictorSPX(26);

  /** Creates a new Shooter. */
  public Shooter() {}
  
  public void setShooterVolts(double leftVolt,double rightVolt){
    sol_atis.setVoltage(leftVolt);
    sag_atis.setVoltage(rightVolt);
  }

  public Command setShooterVoltsCommand(DoubleSupplier leftVolt, DoubleSupplier rightVolt) {
    return runOnce(() -> setShooterVolts(leftVolt.getAsDouble(), rightVolt.getAsDouble()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
