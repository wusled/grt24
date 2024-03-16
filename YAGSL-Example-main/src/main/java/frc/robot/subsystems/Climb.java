// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
 
  Compressor c = new Compressor(PneumaticsModuleType.CTREPCM);
//Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
 // Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
DoubleSolenoid j = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 7);


  /** Creates a new Climb. */
  public Climb() {}

   public void setDoublsolenoid(){
    j.set(Value.kForward);
  }
  public void closeDouble(){
    j.set(Value.kReverse);
  }

 /**  public void setClimb(){
    solenoid1.set(true);
    solenoid2.set(false);
  }
  public void closeClimb(){
    solenoid1.set(false);
    solenoid2.set(true);
  } */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
