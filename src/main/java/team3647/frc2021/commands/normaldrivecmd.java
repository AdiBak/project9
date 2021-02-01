// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2021.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2021.subsystems.Drivetrain;

public class normaldrivecmd extends CommandBase {
  /** Creates a new normaldrivecmd. */
  Drivetrain drivetrain;
  Joystick joystick = new Joystick(0);
  public normaldrivecmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(joystick.getX(), joystick.getX(), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
