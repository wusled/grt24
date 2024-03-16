// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbOpenLoop extends Command {
  private final Climb mClimb;

  /** Creates a new Climb. */
  public ClimbOpenLoop(Climb mClimb) {
    this.mClimb = mClimb;
    addRequirements(mClimb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimb.setDoublsolenoid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimb.closeDouble();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}