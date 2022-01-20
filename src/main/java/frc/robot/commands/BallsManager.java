/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Balling;

public class BallsManager extends CommandBase {
  /**
   * Creates a new BallsManager.
   */
  private enum Action {
    throwing, colloecting, normal, release, collecShoot, slowShoot;
  }

  private JoystickButton aButton;
  private JoystickButton hButton;
  private JoystickButton climbButton;
  private JoystickButton unContainShootButton;
  private XboxController subController;
  private int actionCounter;
  private Balling balls;
  private Action mode = Action.normal;

  public BallsManager(Balling balls, XboxController driver, XboxController subDriver) {
    this.balls = balls;
    aButton = new JoystickButton(driver, 1);
    hButton = new JoystickButton(driver, 4);
    unContainShootButton = new JoystickButton(driver, 3);
    climbButton = new JoystickButton(subDriver, 8);
    subController = subDriver;
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balls.stopMotors();
    balls.low();
    balls.contain();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    actionCounter++;
    switch (mode){
      case normal:
        if(actionCounter==20){
          balls.offSolenoid();
        }
        if(hButton.get()){
          balls.close();
          //balls.offSolenoid();
        }
        
        if(aButton.get()){
          mode = Action.colloecting;
          balls.collect();
          actionCounter=0;
        }
        else if(unContainShootButton.get()){
          balls.low();
          
          mode = Action.collecShoot;
          actionCounter = 0;
        }
        break;
      case throwing:
        if(actionCounter==50){
          balls.unContain();
        }
        if(actionCounter>=70){
          mode = Action.normal;
          actionCounter=0;
          balls.stopMotors();
        }
        break;
      case colloecting:
        if(!aButton.get()&&!(subController.getTriggerAxis(Hand.kRight)>0.3)){
          mode = Action.normal;
          actionCounter=0;
          balls.stopMotors();
        }
        break;
      case release:
        if(!(subController.getPOV()>130&&subController.getPOV()<230)){
          mode = Action.normal;
          actionCounter=0;
          balls.stopMotors();
        }
        break;
      case collecShoot:
        if (actionCounter == 20){
          
          balls.shoot();
        }
        
        else if (actionCounter == 40){
          mode = Action.normal;
          balls.contain();
          balls.stopMotors();
        }
        else if (actionCounter == 30){
          balls.unContain();
        }
        break;
      case slowShoot:
        if(actionCounter == 35){
          balls.unContain();
        }
        if(actionCounter>=50){
          mode = Action.normal;
          actionCounter=0;
          balls.stopMotors();
        }
        break;
    }

  }

  public void collect(){
    mode = Action.colloecting;
    balls.subCollect();
    actionCounter=0;
  }
  public void shoot(){
    balls.low();
    mode = Action.collecShoot;
    actionCounter = 0;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
