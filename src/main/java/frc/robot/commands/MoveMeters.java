package frc.robot.commands;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will turn the robot to the specified angle. */

public class MoveMeters extends RamseteCommand {
    static SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
    
   static  DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        simpleMotorFeedforward, 
        DriveConstants.kDriveKinematics, 1);
    static DifferentialDriveKinematicsConstraint kinematicsConstraint = new DifferentialDriveKinematicsConstraint(
        DriveConstants.kDriveKinematics, 0.1);

        static TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics).addConstraint(voltageConstraint).addConstraint(kinematicsConstraint);

        static PIDController pidController = new PIDController(0.1,0,0);
    /**
     * Turns to robot to the specified angle.
     * @param meters Amount of meters to travel
     * @param drive              The drive subsystem to use
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
