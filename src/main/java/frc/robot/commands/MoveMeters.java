package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will move the robot forward for the specified amount of meeters. */

public class MoveMeters extends RamseteCommand {
    //feed foward class for autonomous command
    static SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
        DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter);
    //pid class for autonomous command
    static PIDController pidController = new PIDController(
        AutoConstants.kTurnP,
        AutoConstants.kTurnI,
        AutoConstants.kTurnD);
    //constraints so it doesnt have a robo-lution
    static  DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        simpleMotorFeedforward, 
        DriveConstants.kDriveKinematics, 
        1);
    static DifferentialDriveKinematicsConstraint kinematicsConstraint = new DifferentialDriveKinematicsConstraint(
        DriveConstants.kDriveKinematics, 
        0.1);
    //config for the ramsete command 
    static TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics)
        .addConstraint(voltageConstraint)
        .addConstraint(kinematicsConstraint);
    /**
     * Moves robot to specified distance.
     * @param meters Amount of meters to travel
     * @param drive  The drive subsystem to use
     */
    public MoveMeters(double meters, DriveSubsystem drive) {
        super(TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), List.of(new Translation2d(meters, 0)),
            new Pose2d(meters, 0, new Rotation2d(0)), config), 
        drive::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
        simpleMotorFeedforward, 
        DriveConstants.kDriveKinematics, 
        drive::getWheelSpeeds, 
        pidController, 
        pidController, 
        drive::tankDriveVolts,drive);
    }
}
