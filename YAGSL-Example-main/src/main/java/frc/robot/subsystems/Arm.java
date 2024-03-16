// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final WPI_VictorSPX kol = new WPI_VictorSPX(14);
  

  /** Creates a new Arm. */
  public Arm() {}
  public void ArmDown( ){
    kol.set(ControlMode.PercentOutput,0.5);  
  }
  public void close(){
    kol.set(ControlMode.PercentOutput, 0);
  }
  public void ArmUp(){
    kol.set(ControlMode.PercentOutput, -1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
