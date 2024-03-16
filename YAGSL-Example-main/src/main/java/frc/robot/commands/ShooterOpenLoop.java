// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShooterOpenLoop extends SequentialCommandGroup {
  private final Shooter mShooter;
  private final Intake mIntake;

  /** Creates a new ShooterOpenLoop. */
  public ShooterOpenLoop(Shooter mShooter, Intake mIntake, double leftVolt, double rightVolt) {
    this.mShooter = mShooter;
    this.mIntake = mIntake;

    addRequirements(mShooter);
    addRequirements(mIntake);

    addCommands(
      mShooter.setShooterVoltsCommand(() -> leftVolt, () -> rightVolt),
      new WaitCommand(2),
      mIntake.setIntakeVoltsCommand(() -> -12)
     
    );
  }
}
