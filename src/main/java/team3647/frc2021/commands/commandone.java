// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2021.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2021.subsystems.Drivetrain;

public class commandone extends CommandBase {
  private final Drivetrain drivetrain;
    Joystick joy1 = new Joystick(0);
  private int driveDistance;
  private double distanceToDrive;

  /**
   * Creates a new commandone.
   * 
   * @param drive
   */
  public commandone(Drivetrain drive, int feet, int inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    addRequirements(drive);
    driveDistance = inches + (12 * feet);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceToDrive = drivetrain.getLeftPosition() + driveDistance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(3, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getLeftPosition() >= distanceToDrive;
  }
}

