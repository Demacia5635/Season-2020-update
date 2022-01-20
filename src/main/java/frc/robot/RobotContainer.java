/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.commands.TelescopeCommand;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.RobotA;
import frc.robot.Constants.RobotB;
import frc.robot.commands.BallsManager;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DrumCommand;
import frc.robot.subsystems.Balling;
import frc.robot.subsystems.Climb;
import frc.robot.commands.OurRamseteCommand;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // private final Roulette m_roulette = new Roulette();
  public final Climb m_climb = new Climb();
  private final Balling m_balling = new Balling();

  public enum DrivingMethods {
    JS_1, JS_2, XBOX_2Sticks, XBOX_1Stick, XBOX_Orbit, Einziger, YorgenMode;
  }

  private Compressor compressor;
  private final XboxController mainController = new XboxController(Constants.mainController);
  private final Chassis chassis = new Chassis(mainController);
  private final XboxController secondaryController = new XboxController(Constants.secondaryController);
  SendableChooser<CommandBase> autoChooser = new SendableChooser<>();
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  // SendableChooser<DrivingMethods> method_chooser = new SendableChooser<>();
  private Command autCommand;
  private final BallsManager ballingCommand = new BallsManager(m_balling, mainController, secondaryController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("Calibrate", new Calibrate(chassis));
    if (Constants.isRobotA) {
      compressor = new Compressor(RobotA.pcmPort);
    } else {
      compressor = new Compressor(RobotB.pcm1Port);
    }
    //autCommand = new InstantCommand(() -> {chassis.setVelocityOurFF(-1, -1);}).andThen(new WaitCommand(20.)).andThen(new InstantCommand(() -> {chassis.setVelocity(0,0);}));
    
    m_climb.drumSubsystem.setDefaultCommand(new DrumCommand(m_climb, secondaryController));
    m_climb.telescopeSubsystem.setDefaultCommand(new TelescopeCommand(m_climb, secondaryController));

    // Configure the button bindings
    configureButtonBindings();
    /*
     * m_chooser.setDefaultOption("Power", new PowerDrive(chassis, inputDevice));
     * m_chooser.addOption("Velocity", new VelocityDrive(chassis, inputDevice));
     * SmartDashboard.putData("Driving Type", m_chooser);
     */
    // SmartDashboard.putData("go2Pose", new InstantCommand(() ->{
    //   chassis.resetOdometry(new Pose2d(3 , -2, Rotation2d.fromDegrees(180)));
    // }, chassis).andThen(new Go2Pose(1, new Translation2d(2, -2), chassis, true)));
    // SmartDashboard.putData("test reset gyro" , new InstantCommand(() ->{
    //   chassis.resetGyro(90);
    // }, chassis));
    // SmartDashboard.putData("go straight", getMovingStraightCommand());
    // SmartDashboard.putData("turn by degrees", getTurnByDegreesCommand());
    // SmartDashboard.putData("angle test", new AngleTest(chassis));
    // SmartDashboard.setDefaultString("Path Following", "Test");
    // SmartDashboard.putData("Path Follow test", getPathFollowingCommand());

    /*
     * method_chooser.setDefaultOption("2 Joystick", DrivingMethods.JS_2);
     * method_chooser.addOption("1 Joystick", DrivingMethods.JS_1);
     * method_chooser.addOption("xbox 1 stick", DrivingMethods.XBOX_1Stick);
     * method_chooser.addOption("xbox 2 sticks", DrivingMethods.XBOX_2Sticks);
     * method_chooser.addOption("XBOX - ORBIT - ULTRA - MODE",
     * DrivingMethods.XBOX_Orbit); method_chooser.addOption("Yorgen Mode",
     * DrivingMethods.YorgenMode); method_chooser.addOption("Einziger",
     * DrivingMethods.Einziger); SmartDashboard.putData("Driving method",
     * method_chooser);
     */
  }

  /*
   * public void setDrivinigMethod(DrivingMethods method) { switch (method) { case
   * JS_2: inputDevice = new InputDev(left, right); break; case JS_1: inputDevice
   * = new InputDev(left); break; case XBOX_2Sticks: inputDevice = new
   * InputDev(mainController, false, false); break; case XBOX_1Stick: inputDevice
   * = new InputDev(mainController, true, false); break; case XBOX_Orbit:
   * inputDevice = new InputDev(mainController, true, true); break; case
   * YorgenMode: inputDevice = new InputDev(mainController,
   * DrivingMethods.YorgenMode); break; case Einziger: inputDevice = new
   * InputDev(mainController, DrivingMethods.Einziger); break; } }
   * 
   * public InputDev getInputDevice() { return inputDevice; }
   * 
   * public DrivingMethods getDrivingMethod() { return
   * inputDevice.getDrivingMethod(); }
   */

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // general
    final JoystickButton backButtonMain = new JoystickButton(mainController, 7);
    final JoystickButton startButtonMain = new JoystickButton(mainController, 8);
    final JoystickButton backButtonSecondary = new JoystickButton(secondaryController, 7); // abort mission
    final JoystickButton startButtonSecondary = new JoystickButton(secondaryController, 8);
    

    startButtonSecondary.whenHeld(new InstantCommand(() -> {
      m_climb.setInClimbing(!m_climb.getInClimbing());
      if(!m_climb.getInClimbing()){
        ballingCommand.schedule();
      }
      else{
        ballingCommand.cancel();
      }
    }));




     backButtonSecondary.whenPressed(new InstantCommand(() -> {
       if (compressor.getClosedLoopControl()) {
         compressor.stop();
       } else {
         compressor.start();
       }
     }));

     startButtonMain.whenPressed(new InstantCommand(() -> {
       if (compressor.getClosedLoopControl()) {
         compressor.stop();
       } else {
         compressor.start();
       }
     }));
     /*bButtonMain.whenHeld(new StartEndCommand(() -> {
      chassis.setVisionMode();
     }, () -> {
      chassis.setVisionMode();
     }));*/
     backButtonMain.whenPressed(new InstantCommand(() -> {
       chassis.setReverse(!chassis.isReversed);
     }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*
     * return new InstantCommand(() -> { chassis.setVelocity(0.5, 0.5);
     * }).withTimeout(2).andThen(new InstantCommand(() -> { chassis.setVelocity(0,
     * 0); }));
     */
    // m_climb.setInClimbing(false);
    // return new turnWhileMoving(90, 1, chassis);
    autCommand = new DriveForward(1.0, chassis, -1.0);//.andThen(ballingCommand.withTimeout(6).alongWith(new WaitCommand(1).andThen((new InstantCommand(ballingCommand::shoot)))));
    return autCommand;
    //return null;
  }

  public void startTeleop() {
    m_climb.setInClimbing(false);
    ballingCommand.schedule();

    /*
     * setDrivinigMethod(method_chooser.getSelected()); if (teleopCommand instanceof
     * PowerDrive) { ((PowerDrive) teleopCommand).setInputDevice(inputDevice); }
     * else { ((VelocityDrive) teleopCommand).setInputDevice(inputDevice); } return
     * teleopCommand;
     */
  }

  public void enableInit() {
    //chassis.resetOdometry();
  }
  /**
   * Attempts the AutoNav challenge.
   * Opens the json file of the selected path, and follows it.
   * 
   * @return the AutoNav command
   */
  private Command getBounceCommand() {
    final String[] bouncers = new String[] { "paths/output/bouncer1.wpilib.json",
                                             "paths/output/bouncer2.wpilib.json",
                                             "paths/output/bouncer3.wpilib.json",
                                             "paths/output/bouncer4.wpilib.json" };

    Trajectory[] trajectory = new Trajectory[4];

    for (int i = 0; i < bouncers.length; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(bouncers[i]);
        trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + bouncers[i], ex.getStackTrace());
      }
    }

    Command cmd = null;
    for (Trajectory t : trajectory) {
      if (cmd == null) {
        cmd = new OurRamseteCommand(t, chassis::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            Constants.DRIVE_KINEMATICS, chassis::setVelocityOurFF, chassis);
      } else {
        cmd = cmd.andThen(chassis.getReverseCommand())
            .andThen(new OurRamseteCommand(t, chassis::getPose,
                new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                Constants.DRIVE_KINEMATICS, chassis::setVelocityOurFF, chassis));
      }
    }
    return cmd.andThen(() -> chassis.setVelocity(0, 0));
  }

  private Command getAutoNavCommand() {
    final String path = "paths/output/Autonomous_Red.wpilib.json";

    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return new OurRamseteCommand(trajectory, chassis::getPose,
          new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
          Constants.DRIVE_KINEMATICS, chassis::setVelocityOurFF, chassis)
              .andThen(() -> chassis.setVelocity(0, 0));
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
    }

    return null;
  }

  /**
   * Attempts the Galactic Search challenge.
   * Detects whether the path is path A or B and whether it is the red or blue
   * path.
   * Afterwards it finds each power cell, drives to it, and picks it up.
   * 
   * @return the Galactic Search command
   */
  private Command getGalacticSearchCommand() {
    return SequentialCommandGroup.sequence(/* new ArmChange(ArmChange.Position.Bottom, pickup), */
        new RunCommand(ballingCommand::collect), new SelectCommand(() -> {
          double angleToNearestBall = SmartDashboard.getNumber("BallAngle", 0);
          double distanceToNearestBall = SmartDashboard.getNumber("BallDistance", 0);

          // Path A
          if (Math.abs(angleToNearestBall) <= 2) {
            // Red path
            if (distanceToNearestBall < Constants.CHALLENGE_SPACE_WIDTH / 2) {
              return SequentialCommandGroup.sequence(
                  chassis.driveToBallCommand(Constants.MAX_AUTOMATION_VELOCITY),
                  chassis.findAndDriveToBall(true), chassis.findAndDriveToBall(false));
            }

            // Blue path
            return SequentialCommandGroup.sequence(
                chassis.driveToBallCommand(Constants.MAX_AUTOMATION_VELOCITY),
                chassis.findAndDriveToBall(false), chassis.findAndDriveToBall(false));
          }

          // Path B

          // Red path
          if (distanceToNearestBall * Math.cos(Math.toRadians(angleToNearestBall)) <= 140
              * Constants.INCHES_TO_METERS) {
            return SequentialCommandGroup.sequence(chassis.findAndDriveToBall(true),
                chassis.findAndDriveToBall(false), chassis.findAndDriveToBall(false));
          }

          // Blue path
          return SequentialCommandGroup.sequence(chassis.findAndDriveToBall(true),
              chassis.findAndDriveToBall(false), chassis.findAndDriveToBall(true));
        }) /* , new ArmChange(ArmChange.Position.Top, pickup) */);
  }

  public static int getGalacticPath() {
    double angleToNearestBall = SmartDashboard.getNumber("BallAngle", 0);
    double distanceToNearestBall = SmartDashboard.getNumber("BallDistance", 0);

    if (distanceToNearestBall == 0) {
      return 0;
    }

    // Path A
    if (Math.abs(angleToNearestBall) <= 2) {
      if (distanceToNearestBall < Constants.CHALLENGE_SPACE_WIDTH / 2) {
        return 1;
      }

      // Blue path
      return 2;
    }

    // Path B

    // Red path
    if (distanceToNearestBall * Math.cos(Math.toRadians(angleToNearestBall)) <= 140
        * Constants.INCHES_TO_METERS) {
      return 3;
    }

    // Blue path
    return 4;

  }

  public double getAngle() {
    return SmartDashboard.getNumber("ShootAngle", 5);
  }

  public double getVel() {
    return SmartDashboard.getNumber("ShootVel", 4500);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command[] getAutonomousCommands() {
    return new Command[] {
                           autCommand
                            };
  }

  public Command[] getTeleopCommands() {
    return new Command[] { new InstantCommand(chassis::resetAngle) ,(new InstantCommand(() -> chassis.resetOdometry(new Pose2d(9.14-2.90, 2.25, Rotation2d.fromDegrees(0))))) };
  }
}
