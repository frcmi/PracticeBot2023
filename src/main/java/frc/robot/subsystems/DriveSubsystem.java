// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;

public class DriveSubsystem extends SubsystemBase {
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
  // The motors on the left side of the drive.
  WPI_TalonFX front_left = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  WPI_TalonFX back_left = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(front_left, back_left);
  // The motors on the right side of the drive.
  WPI_TalonFX front_right = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  WPI_TalonFX back_right = new WPI_TalonFX(DriveConstants.kRightMotor2Port);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(front_right, back_right);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


  // The gyro sensor (Pigeon 2)
  private final AHRS navX = new AHRS();
  // private final  pigeon = new WPI_Pigeon2(DriveConstants.kPigeonPort);
  private final Gyro m_gyro = navX;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  //Field2d Sim
  private final Field2d m_field = new Field2d();

  //Some smartdashboard variables
  private double y_Displacement = 0.0; 
  private double x_Displacement = 0.0; 
  private final double two = 2.0; 
  private String i_j_Displacement = "";


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    front_left.configFactoryDefault();
    back_left.configFactoryDefault();
    front_right.configFactoryDefault();
    back_right.configFactoryDefault();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotors.setInverted(true);
    SmartDashboard.putData("Field", m_field);

    back_left.follow(front_left);
    back_left.setInverted(InvertType.FollowMaster);
    back_right.follow(front_right);
    back_right.setInverted(InvertType.FollowMaster);
    // Sets up the encoders
    front_left.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    back_left.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    front_right.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    back_right.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);

    resetEncoders();
    zeroHeading();
    m_odometry = new DifferentialDriveOdometry(getHeading(), 0, 0, new Pose2d()); 
  }

  @Override
  public void periodic() {
    m_drive.feed();
    // Update the odometry in the periodic block
    m_odometry.update(
        getHeading(), 
        getLeftEncoderDistance(), 
        getRightEncoderDistance());

    Pose2d pose = m_odometry.getPoseMeters();
    m_field.setRobotPose(pose);


    SmartDashboard.putNumber("True Left Volt", front_left.get());
    SmartDashboard.putNumber("True Right Volt", front_right.get());
    SmartDashboard.putNumber("Left Curent", front_left.getStatorCurrent());
    SmartDashboard.putNumber("Right Current", front_right.getStatorCurrent());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder", getRightEncoderDistance());
    // x_Displacement = m_odometry.getPoseMeters().getX(); 
    // y_Displacement = m_odometry.getPoseMeters().getY(); 
    // i_j_Displacement = x_Displacement + "i + " + y_Displacement + "j"; 
   
    // SmartDashboard.putString("Polar Displacement", "[" + Math.sqrt(Math.pow(x_Displacement, 
    // 2) + Math.pow(y_Displacement, two)) + " " 
    // + Math.atan(y_Displacement/x_Displacement) + "]");

    // SmartDashboard.putString("Rectangular Displacement", i_j_Displacement);

  }

  public double getLeftEncoderDistance() {
    return -front_left.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;
  }

  public double getRightEncoderDistance() {
    return back_right.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;
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
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() { //in m/s
    return new DifferentialDriveWheelSpeeds(
    -front_left.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse * 10, 
    front_right.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse * 10);
  }

  public CommandBase resetOdometryCmd(Pose2d pose) {
    return Commands.runOnce(() -> resetOdometry(pose), this);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getHeading(), 0, 0, pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public CommandBase stop() {
    return Commands.runOnce(m_drive::stopMotor, this);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("Set Left Volt", leftVolts);
    SmartDashboard.putNumber("Set Right Volt", rightVolts);
    
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
   front_left.setSelectedSensorPosition(0.0);
   back_left.setSelectedSensorPosition(0.0);
   front_right.setSelectedSensorPosition(0.0);
   back_right.setSelectedSensorPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d() /* .plus(Rotation2d.fromDegrees(180))*/;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  /**
   * Feeds the motors
   * 
   * @return nothing this is void lol
   * @param no there is none smh
   */
  public void feed() {
    front_right.feed();
    front_left.feed();
    back_right.feed();
    back_left.feed();
  }
  public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
    return followTrajectoryCommand(traj, false);
  }
  
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    final PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    new PathConstraints(0.1, 0.05), 
        new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading
        new PathPoint(new Translation2d(1, 3.0), Rotation2d.fromDegrees(0)) // position, heading
    );
    return new SequentialCommandGroup(
        runOnce(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj1.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj1, 
            this::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            this::getWheelSpeeds,
            new PIDController(AutoConstants.kTurnP, 0, 0),
            new PIDController(AutoConstants.kTurnP, 0, 0),
            // RamseteCommand passes volts to the callback
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
  }
}

