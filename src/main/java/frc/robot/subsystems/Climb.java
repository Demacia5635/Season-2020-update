/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.RobotA;
import frc.robot.Constants.RobotB;

public class Climb extends SubsystemBase {

  public SubsystemBase drumSubsystem = new SubsystemBase() {
  };
  public SubsystemBase telescopeSubsystem = new SubsystemBase() {
  };

  public DoubleSolenoid lockPiston;

  public boolean inClimbing = false;

  public TalonSRX telescopicMotor;

  public TalonSRX drumMotor;

  public Climb() {

    if (Constants.isRobotA) {
      lockPiston = new DoubleSolenoid(RobotA.pcmPort, RobotA.fSolenoid_lock, RobotA.rSolenoid_lock);
    } else {
      lockPiston = new DoubleSolenoid(RobotB.pcm2Port, RobotB.fSolenoid_lock, RobotB.rSolenoid_lock);
    }

    telescopicMotor = new TalonSRX(Constants.telescopicMotor);
    telescopicMotor.configContinuousCurrentLimit(15);
    telescopicMotor.enableCurrentLimit(true);
    drumMotor = new TalonSRX(Constants.drumMotor);
  }

  // Returns if in climbing mode
  public boolean getInClimbing() {
    return inClimbing;
  }

  // Sets if/not in climbing mode
  public void setInClimbing(boolean inClimbing) {
    this.inClimbing = inClimbing;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Is in climbing", this::getInClimbing, this::setInClimbing);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets power to the telescopic elevator motor
  public void setTelescopicMotor(double power) {
    if (power != 0)
      System.out.println("Telescopic Power = " + -power);
    telescopicMotor.set(ControlMode.PercentOutput, -power);
  }

  // Locks the drum lock piston
  public void lockLockPiston() {
    Value value = Constants.isRobotA ? Value.kReverse : Value.kForward;
    System.out.println("Locking");
    lockPiston.set(value);
  }

  // Releases the drum lock piston
  public void releaseLockPiston() {
    Value value = Constants.isRobotA ? Value.kForward : Value.kReverse;
    System.out.println("Unlocking");
    lockPiston.set(value);
  }

  // Turns of the drum lock piston
  public void turnOffLockPiston() {
    System.out.println("Turning off lock");
    lockPiston.set(Value.kOff);
  }

  // Sets power to the drum motor
  public void setDrumMotor(double power) {
    if (power != 0)
      System.out.println("Drum Power: " + -power);
    drumMotor.set(ControlMode.PercentOutput, -power);
  }
}
