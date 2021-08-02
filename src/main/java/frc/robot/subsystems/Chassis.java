/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // import the tlaonFX
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Calibrate;
import frc.robot.commands.Drive;
import frc.robot.utils.FeedForward;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase {

  // TO DO: check the engines direction, maybe invert
  private final DifferentialDriveOdometry m_odometry;
  private GroupOfMotors right;
  private GroupOfMotors left;
  private PigeonIMU gyro;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.CHASSIS_KS, Constants.CHASSIS_KV, Constants.CHASSIS_KA);
  private final Field2d field = new Field2d();
  private boolean isBrake = true;
  public boolean isReversed = false; 
  private XboxController mainController;
  private Drive driveCommand;

  /**
   * Creates a new Chassis.
   */
  public Chassis(XboxController mainController) {
    SmartDashboard.putBoolean("IsPower", false);

    WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.RIGHT_FRONT);
    WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.LEFT_FRONT);
    WPI_TalonSRX rightBack = new WPI_TalonSRX(Constants.RIGHT_BACK);
    WPI_TalonSRX leftBack = new WPI_TalonSRX(Constants.LEFT_BACK);

    //leftFront.setSensorPhase(true);
    //rightFront.setSensorPhase(false);
    
    this.mainController = mainController;
    this.right = new GroupOfMotors(rightFront, rightBack);
    this.left = new GroupOfMotors(leftFront, leftBack);
    left.invertMotors();
    right.invertMotors();
    this.gyro = new PigeonIMU(Constants.gyroPort);
    left.resetEncoder();
    right.resetEncoder();
    setMotorNeutralMode(isBrake);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getFusedHeading()));

    SmartDashboard.putData("Field", field);

    driveCommand = new Drive(this, mainController);
    setDefaultCommand(driveCommand);
  }

  public void setVelocity(double left, double right) {
    this.left.setVelocity(left, feedforward);
    this.right.setVelocity(right, feedforward);
  }

  public void setVisionMode(){
    driveCommand.setVisionMode();
  }

  public double getFusedHeading() {
    double res = 0;
    if (gyro != null) {
      res =  gyro.getFusedHeading();
    } else {
      
      if (gyro != null) {
        res =  gyro.getFusedHeading();
      }
    }
    if (isReversed) return res + 180.0; 
    return res; 
  }

  public double getAngle() {
    double angle = getFusedHeading();

    if (angle < 0) {
      angle = -((-angle) % 360.0);
      if (angle < -180) {
        return 360.0 + angle;
      }
      return angle;
    }
    angle = angle % 360.0;
    if (angle > 180) {
      return angle - 360.0;
    }
    return angle;
  }

  public double getRightPos() {
    if (isReversed) return -left.getDistance();
    return right.getDistance();
  }

  public double getLeftPos() {
    if (isReversed) return -right.getDistance();
    return left.getDistance();
  }

  public double getRightVelocity() {
    if (isReversed) return -left.getVelocity();
    return right.getVelocity();
  }

  public double getLeftVelocity() {
    if (isReversed) return -right.getVelocity();
    return left.getVelocity();
  }

  public double getPos() {
    return (getLeftPos() + getRightPos())/2.0;
  }

  /**
   * gets 2 values between 1 to -1 one to determine the tangent velocity
   * and the other determines the radial accelaration of the robot
   * 
   * the function sets calculated values for the right and left motors
   */
  public void radialAccelaration(double velocity, double turns) {
    velocity = velocity * Constants.MAX_VELOCITY;
    turns = turns * Constants.MAX_RADIAL_ACCELARATION;
    double right = 0;
    double left = 0;
    if (velocity != 0) {
      if (turns > 0) {
        double radius = (velocity * velocity / turns);
        right = (velocity / radius) * (radius - (Constants.ROBOT_TRACK_WIDTH / 2));
        left = (velocity / radius) * (radius + (Constants.ROBOT_TRACK_WIDTH / 2));
      } else if (turns < 0) {
        double radius = (velocity * velocity / (-turns));
        right = (velocity / radius) * (radius + (Constants.ROBOT_TRACK_WIDTH / 2));
        left = (velocity / radius) * (radius - (Constants.ROBOT_TRACK_WIDTH / 2));
      } else {
        right = velocity;
        left = velocity;
      }
    } else {
      if (turns > 0) {
        right = -Math.sqrt(turns * (Constants.ROBOT_TRACK_WIDTH / 2));
        left = Math.sqrt(turns * (Constants.ROBOT_TRACK_WIDTH / 2));
      } else {
        right = Math.sqrt((-turns) * (Constants.ROBOT_TRACK_WIDTH / 2));
        left = -Math.sqrt((-turns) * (Constants.ROBOT_TRACK_WIDTH / 2));
      }
    }
    setVelocity(left, right);
  }

  public double getVisionAngle(){
    return SmartDashboard.getNumber("VisionAngle", 0);
  }

  public double getVisionDistance(){
    return SmartDashboard.getNumber("VisionDistance", 0);
  }

  public void gotToBall(double speed){// -1 to 1
    double angle = getVisionAngle();
    double distance = getVisionDistance();

    if (angle == 0){
      setVelocityOurFF(speed, speed);
      return;
    }

    double radius = distance / (2 * Math.sin(Math.toRadians(angle)));

    double k = Constants.robotTrackWidth / 2;

    double left;
    double right;

    if (angle < 0){
      left = speed;
      right = speed * (radius + k) / (radius - k);
    } 
    else {
      right = speed;
      left = speed * (radius - k) / (radius + k);
    }

    left *= Constants.MAX_VELOCITY;
    right *= Constants.MAX_VELOCITY;

    setVelocityOurFF(left, right);
  } 

  /**
   * gets 2 values between 1 to -1 one to determine the tangent velocity
   * and the other determines the angular velocity of the robot
   * 
   * the function sets calculated values for the right and left motors
   */
  public void angularVelocity(double velocity, double turns) {
    if (SmartDashboard.getBoolean("IsPower", false)){
      double left = velocity - turns;
      double right = velocity + turns;
      left = Math.max(left, -1);
      left = Math.min(left, 1);
      right = Math.max(right, -1);
      right = Math.min(right, 1);
      setPower(left, right);
    } else {
      velocity = velocity * Constants.MAX_VELOCITY;
      turns = turns * Constants.MAX_ANGULAR_VELOCITY;
      double right = 0;
      double left = 0;
      //if (velocity >= 0) {
      //  right = velocity - turns * (Constants.ROBOT_TRACK_WIDTH / 2);
      //  left = velocity + turns * (Constants.ROBOT_TRACK_WIDTH / 2);
      //} else {
        right = velocity + turns * (Constants.ROBOT_TRACK_WIDTH / 2);
        left = velocity - turns * (Constants.ROBOT_TRACK_WIDTH / 2);
      //}
      if (velocity != 0){
        System.out.println("velocity: " + velocity);
        System.out.println("turns: " + turns);
        System.out.println("left: " + left);
        System.out.println("right: " + right);
      }
      if (velocity == 0 && turns != 0){
        setVelocityOurFF(-1. * turns, 1. * turns);
      } else{
        setVelocityOurFF(left, right);
      }
    }
    
    
    
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    this.left.resetEncoder();
    this.right.resetEncoder();
    
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getFusedHeading()));
  }

  public double get_ks() {
    return Constants.CHASSIS_KS;
  }

  public double get_kv() {
    return Constants.CHASSIS_KV;
  }

  public double get_kp() {
    return Constants.CHASSIS_KP;
  }

  public double SpeedInMtoSec1() {
    return this.getLeftVelocity() * 10 / Constants.PULSES_PER_METER;
  }

  public double SpeedInMtoSec2() {
    return this.getRightVelocity() * 10 / Constants.PULSES_PER_METER;
  }

  /**
   * Drives to the ball on an arc
   * 
   * @param speed - The velocity at which the robot will drive in Meters
   */
  public void driveToBall(double speed) {
    double distance = SmartDashboard.getNumber("BallDistance", 0) / 100.0;
    double angle = SmartDashboard.getNumber("BallAngle", 0);
    angle = angle + 6.1155;
    double difFromMid = 0.15;
    double lDistance = distance;
    distance = Math.sqrt(Math.pow(distance, 2) + Math.pow(difFromMid, 2)
        - 2 * distance * difFromMid * Math.cos(Math.toRadians(-angle + 90)));
    //angle = -((Math.PI / 2) - Math.asin(lDistance * Math.sin(Math.toRadians(-angle + 90)) / distance));
    angle = Math.toRadians(angle) - Math.acos((Math.pow(difFromMid, 2) - Math.pow(lDistance, 2) - Math.pow(distance, 2)) / (-2 * distance * lDistance));
    double radius = distance / (2 * Math.sin(angle));
    double k = Constants.ROBOT_TRACK_WIDTH / 2;
    double left = speed * (1 + (k / radius));
    double right = speed * (1 - (k / radius));
    System.out.println(left);
    System.out.println(right);
    setVelocity(left, right);
  }

  /**
   * 
   * @param speed - The velocity at which the robot will drive in Meters
   * 
   * @return A command that will execute the driveToBall function pereiodecly
   *         untill reacing the ball
   */
  public CommandBase driveToBallCommand(double speed) {
    return new FunctionalCommand(() -> {
    }, () -> {
      driveToBall(speed);
    }, (interrupted) -> {
      setVelocity(0, 0);
    }, () -> {
      return SmartDashboard.getNumber("BallDistance", 0) == 0;
    });
  }

  /**
   * Finds the nearest ball and drives to it.
   * 
   * @param isClockwise defines whether the robot would look for the ball by
   *                    turning clockwise or counterclockwise.
   * 
   * @return the command that finds, and drives to the ball
   */
  public Command findAndDriveToBall(boolean isClockwise) {
    return SequentialCommandGroup.sequence(new RunCommand(() -> {
      setVelocity((isClockwise ? 1 : -1) * Constants.MAX_AUTOMATION_VELOCITY / 2,
          (isClockwise ? -1 : 1) * Constants.MAX_AUTOMATION_VELOCITY / 2);
    }, this).withInterrupt(() -> (SmartDashboard.getNumber("BallAngle", 0) == 0)),
        driveToBallCommand(Constants.MAX_AUTOMATION_VELOCITY));
  }

  public void setPos(double leftPos, double rightPos) {
    this.left.setMotionMagic(leftPos, this.feedforward, Constants.CRUISE_VELOCITY,
        Constants.ACCELERATION);
    this.right.setMotionMagic(rightPos, this.feedforward, Constants.CRUISE_VELOCITY,
        Constants.ACCELERATION);
  }

  public void setPosWithoutFF(double leftPos, double rightPos) {
    this.left.setMotionMagic(leftPos);
    this.right.setMotionMagic(rightPos);
  }

  public void goTo(double distanceLeft, double distanceRight) {
    // this.configMotionMagic();
    this.setPosWithoutFF(this.left.getEncoder() + distanceLeft, this.right.getEncoder() + distanceRight);
  }

  public void goTo(double distance) {
    // this.configMotionMagic();
    this.setPosWithoutFF(this.left.getEncoder() + distance, this.right.getEncoder() + distance);
  }

  public void configMotionMagic(double accelaration, double curve) {
    for (GroupOfMotors motor : new GroupOfMotors[] { left, right }) {
      motor.setMotionSCurve((int) curve);
      motor.setCruiseVelocity(Constants.CRUISE_VELOCITY);
      motor.setAcceleration(accelaration);
    }
  }

  public void configMotionMagic() {
    configMotionMagic(Constants.ACCELERATION, Constants.MOTION_S_CURVE);
  }

  public void configMotionMagic(double acceleration) {
    configMotionMagic(acceleration, Constants.MOTION_S_CURVE);
  }

  public void configMotionMagic(int curve) {
    configMotionMagic(Constants.ACCELERATION, curve);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(getRotation2d(), getLeftPos(), getRightPos());
    field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void setPower(double left, double right) {
    this.left.setPower(left);
    this.right.setPower(right);
  }

  public double getChassisDistance() {
    return (this.getLeftPos() + this.getRightPos()) / 2;
  }

  /**
   * 
   * @param angle - an angle between 0 to 360
   * 
   * @return - return the angle between 180 to -180
   */
  public double normalizeAngle(double angle) {
    return Math.IEEEremainder(angle, 360);
  }

  public double getNormalizedAngle() { // returns the angle of the robot between 180 to -180
    return normalizeAngle(getAngle());
  }

  /**
   * 
   * @param reqAngle
   * @param curAngle
   * 
   * @return returns the smallest delta between the angles
   */
  public double diffAngle(double reqAngle, double curAngle) { // returns the shortest angle to what
                                                              // you want
    double a1 = normalizeAngle(reqAngle) - normalizeAngle(curAngle);
    if (a1 <= -180) {
      return a1 + 360;
    } else if (a1 > 180) {
      return a1 - 360;
    } else {
      return a1;
    }
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getFusedHeading());//getAngle());
  }

  public double getAngle2Pose(Pose2d pose) {
    Translation2d translation2d = pose.getTranslation().minus(getPose().getTranslation());
    return new Rotation2d(translation2d.getX(), translation2d.getY()).getDegrees();
  }

  public void setReverse(boolean isReversed) {
    if (this.isReversed != isReversed) {
      this.isReversed = isReversed; 
      resetOdometry(getPose());
    }

  }

  public void setReverse() {
    setReverse(!isReversed);
  }

  public void setVelocityOurFF(double left, double right) {
    if (isReversed) {
      setPower(-FeedForward.feedForwardLeftPower(left, right), -FeedForward.feedForwardRightPower(left, right));
      /*this.left.setVelocity(-right, -FeedForward.feedForwardRightPower(left, right));
      this.right.setVelocity(-left, -FeedForward.feedForwardLeftPower(left, right));*/
    }
    else {
      setPower(FeedForward.feedForwardLeftPower(left, right), FeedForward.feedForwardRightPower(left, right));
      /*this.left.setVelocity(left, FeedForward.feedForwardLeftPower(left, right));
      this.right.setVelocity(right, FeedForward.feedForwardRightPower(left, right));*/
    }
  }

  public Command getReverseCommand() {
    return new InstantCommand(() -> setReverse()); 
  }

  public void setAngle(double degrees){
    gyro.setFusedHeading(degrees);
  }

  public void resetAngle(){
    setAngle(0);
  }

  public void setLeftPos(double distance) {
    this.left.setPosition(distance);
  }
  public void setRightPos(double distance) {
    this.right.setPosition(distance);
  }

  public void setLeftPos(double distance, double feedforward) {
    left.setPosition(distance, feedforward);
  }

  public void setRightPos(double distance, double feedforward) {
    right.setPosition(distance, feedforward);
  }

  public void setMotorNeutralMode(boolean isBrake) {
    this.isBrake = isBrake;
    left.setNeutralMode(isBrake);
    right.setNeutralMode(isBrake);
  }

  public void changeMotorsNeutralMode() {
    setMotorNeutralMode(!isBrake);
  }

  public boolean getIsMotorsNeutralModeBrake() {
    return isBrake;
  }
  
  public void Set_K_I(double ki) {
    this.left.setK_I(ki);
    this.right.setK_I(ki);
  }

  public void Set_K_D(double k_d) { 
    this.left.setK_D(k_d);
    this.right.setK_D(k_d);
  }
  public void Set_K_P(double k_p) { 
    this.left.setK_P(k_p);
    this.right.setK_P(k_p);
  }
  public double getLeftMotorOutput() {
    return this.left.getPower(); 
  }

  public double getRightMotorOutput() {
    return this.right.getPower(); 
  }

  public double getLeftClosedLoopError() {
    return this.left.getError(); 
  }

  public double getRightClosedLoopError() {
    return this.right.getError(); 
  }

  public void setRightAllowedError(double error) {
    this.right.setClosedLoopAllowedError(error);
  }
  public void setLeftAllowedError(double error) {
    this.left.setClosedLoopAllowedError(error);
  }

  public void setAllowedError(double error) {
    this.setLeftAllowedError(error);
    this.setRightAllowedError(error);
  }

  public double getX(){
    return getPose().getX();
  }
  
  public double getY(){
    return getPose().getY();
  }

  public void Calibrate(boolean b) {
    try {
      new Calibrate(this).schedule();
    }
    catch (Exception e) {
      System.out.println("ERROR IN CALIBRATE " + e);
    }
  }

  public double getDistanceFromIdk(){
    return 9.14 - getX();
  }

  public Translation2d robotToTarget(){
    Translation2d robotToTarget = new Translation2d(9.14, 2.25).minus(getPose().getTranslation());
    return robotToTarget;
  }

  public double getAngleToTarget(){
    Translation2d robotToTarget = robotToTarget();
    return new Rotation2d(robotToTarget.getX(), robotToTarget.getY()).getDegrees() - getAngle();
  }

  public double getDistanceToTarget(){
    return robotToTarget().getNorm();
  }

  public boolean AlwaysTrue() { return true;  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty(key, getter, setter);
    builder.addDoubleProperty("Left Distance", this::getLeftPos, null);
    builder.addDoubleProperty("Right Distance", this::getRightPos, null);
    builder.addDoubleProperty("Left Encoder", this.left::getEncoder, null);
    builder.addDoubleProperty("Right Encoder", this.right::getEncoder, null);
    builder.addDoubleProperty("Left Speed", this::getLeftVelocity, null);
    builder.addDoubleProperty("Right Speed", this::getRightVelocity, null);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addBooleanProperty("Neutral Mode", this::getIsMotorsNeutralModeBrake, this::setMotorNeutralMode);
    builder.addDoubleProperty("Left Motor power", this:: getLeftMotorOutput, null);
    builder.addDoubleProperty("Right Motor power", this:: getRightMotorOutput, null);
    builder.addDoubleProperty("Pose X", this::getX, null);
    builder.addDoubleProperty("Pose Y", this::getY, null);
    builder.addDoubleProperty("Distance", this::getDistanceFromIdk, null);
    builder.addDoubleProperty("Galactic Path", RobotContainer::getGalacticPath, null);
    // builder.addBooleanProperty("Calibrate", this::AlwaysTrue, this::Calibrate);
    builder.addDoubleProperty("Angle To Target", this::getAngleToTarget, null);
    builder.addDoubleProperty("Distance To Target", this::getDistanceToTarget, null);
    builder.addDoubleProperty("turns", () -> {return mainController.getX(Hand.kLeft);}, null);
    builder.addDoubleProperty("leftTrigger", () -> {return mainController.getTriggerAxis(Hand.kLeft);}, null);
    builder.addDoubleProperty("rightTrigger", () -> {return mainController.getTriggerAxis(Hand.kRight);}, null);
    
  }

}
