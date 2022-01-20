/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final boolean isRobotA = true;

    public final class RobotA {
        // Characteristics
        public static final double kS = 1.;
        public static final double kV = 2.24;
        public static final double kA = 0.252;
        public static final double kP = 0.00843;

        // PCM
        public static final int pcmPort = 20;

        // balling solenoids
        public static final int rMode1 = 0;
        public static final int fMode1 = 1;
        public static final int rMode2 = 3;
        public static final int fMode2 = 2;
        public static final int fContainer = 4;
        public static final int rContainer = 5;
		
		// balling speeds		
    	public static final double superSlowSmallW = 0.4;
    	public static final double superSlowBigW = 0.25;

        // climbing solenoids
        public static final int rSolenoid_lock = 7;
        public static final int fSolenoid_lock = 6;
    }

    public final class RobotB {
        // Characteristics
        public static final double kS = 1.54;
        public static final double kV = 2.15;
        public static final double kA = 1.54;
        public static final double kP = 0.0146;

        // PCMS
        public static final int pcm1Port = 19;
        public static final int pcm2Port = 20;

        // balling solenoids
        public static final int rMode1 = 0;
        public static final int fMode1 = 1;
        public static final int rMode2 = 5;
        public static final int fMode2 = 4;
        public static final int fContainer = 2;
        public static final int rContainer = 3;

		// balling speeds		
    	public static final double superSlowSmallW = 0.2;
    	public static final double superSlowBigW = 0.3;

        // climbing solenoids
        public static final int rSolenoid_lock = 2;
        public static final int fSolenoid_lock = 3;
    }

    public static final int leftJoystick = 1;
    public static final int rightJoystick = 2;
    public static final int mainController = 0;
    public static final int secondaryController = 1;

    public static final int leftFrontMotor = 3;
    public static final int leftBackMotor = 4;
    public static final int rightFrontMotor = 2;
    public static final int rightBackMotor = 1;

    public static final int gyroPort = 11;

    // Balling
    public static final int lWheelsPort = 6;
    public static final int bWheelsPort = 5;
    public static final double fastBigW = 0.8;
    public static final double slowBigW = 0.6;
    public static final double normalBigW = 0.635;

    public static final double fastSmallW = 0.8;
    public static final double slowSmallW = 0.4;
    public static final double normalSmallW = 0.7;

    // Climb
    public static final int telescopicMotor = 10;
    public static final int drumMotor = 9;

    public static final double robotTrackWidth = 0.585;

    public static final double maxRotationV = 3.2;
    public static final double maxAngVel = (2 * Math.PI) - 4;
    public static final double maxVel = 3.5;

    public static final double kD = 0;
    public static final int maxCurrent = 40;
    public static final double wheelDiameter = 0.1524;
    public static final double encoderPulsePerRotation = 800.0;
    public static final double pulsePerMeter = encoderPulsePerRotation / (wheelDiameter * Math.PI);

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            robotTrackWidth);

    //public static final double kMaxSpeedMetersPerSecond = 4.559;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final boolean kGyroReversed = false;
    public static final int SHOOTER_WHEEL_PORT = 10;
    public static final int HOOD_MOTOR_PORT = 8;
    public static final int BONKER_PORT = 7;
    public static final int VACUUM_MOTOR_PORT = 9;
    public static final int LIM_PORT = 0;
    public static final int ROULETTE_MOTOR_PORT = 11;
    public static final int ARM_MOTOR_PORT = 5;
    public static final int PICKUP_MOTOR_PORT = 6;
    public static final double ZERO_SHOOT_DISTANCE = 140;

    public static final double SHOOTER_KS = 1. / 10900.;
    public static final double SHOOTER_KV = 0.09;
    public static final double SHOOTER_KP = 0.05;
    public static final double HOOD_KP = 0.1;
    public static final double HOOD_KI = 0.01;
    public static final double ARM_KP = -1;

    public static final double ROULETTE_ROTATION_PER_SEC = 0.5;

    public static final int RIGHT_FRONT = 2;
    public static final int RIGHT_BACK = 1;
    public static final int LEFT_FRONT = 3;
    public static final int LEFT_BACK = 4;

    public static final int XBOX_PORT = 0;

    public static final double ROBOT_TRACK_WIDTH = 0.59; // In meters
    public static final double MAX_VELOCITY = 4; // In meters per second
    public static final double MAX_RADIAL_ACCELARATION = 4; // In meters per squared second
    public static final double MAX_ANGULAR_VELOCITY = Math.PI ; // In radians per second

    public static final int MAX_ARM_POS = 150;

    public static final double PULSES_PER_METER = 1666;

    public static final int MOTION_S_CURVE = 3; // an integer between 0 - 8 that represents the
    // velocity smoothing
    public static final double ACCELERATION = 10; // the acceletarion in sensor units per 100 ms
    public static final double CRUISE_VELOCITY = 5 * PULSES_PER_METER / 10; // the peak speed in
    // sensor
    // units per 100 ms
    public static final double kS = 1.;
    public static final double kV = 2.24;
    public static final double kA = 0.252;
    public static final double kP = 0.00843;

    public static final double CHASSIS_KP = 0.00843;
    public static final double CHASSIS_KI = 0;
    public static final double CHASSIS_KD = 0;
    public static final double CHASSIS_KV = 0.106; // 0.638;
    public static final double CHASSIS_KS = 0.214; // 2.28;
    public static final double CHASSIS_KA = 0.252;

    /**
     * Galactic Search
     */
    public static final double CHALLENGE_SPACE_WIDTH = 9.15;
    public static final double CHALLENGE_SPACE_HEIGHT = 4.58;
    public static final double MAX_AUTOMATION_VELOCITY = 2;
    public static final double MAX_AUTOMATION_ACCELERATION = 2;

    public static final double INCHES_TO_METERS = 0.0254;

    /**
     * AutoNav
     */

    // DifferentialDriveKinematics
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                                                                     new DifferentialDriveKinematics(
                                                                             ROBOT_TRACK_WIDTH);

    // Ramsete Parameters (in meters and seconds)
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
}
