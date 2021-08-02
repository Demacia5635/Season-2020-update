// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveForward extends CommandBase {
  private double distance;
  private double target = 0;
  private double velocity;
  private Chassis chassis;
  /** Creates a new DriveForward. */
  public DriveForward(double distance, Chassis chassis, double velocity) {
    this.velocity = velocity;
    this.distance = distance;
    this.chassis = chassis;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.setVelocityOurFF(velocity, velocity);
    target = chassis.getPos() + distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(target - chassis.getPos()) < 0.1; 
  }
}
