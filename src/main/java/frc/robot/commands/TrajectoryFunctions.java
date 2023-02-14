package frc.robot.commands;

import java.util.HashMap;

import com.ctre.phoenix.platform.can.AutocacheState;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

public final class TrajectoryFunctions {
    public static PathPlannerTrajectory combineTrajectories(Trajectory... trajectories) {
        PathPlannerTrajectory finalTrajectory = new PathPlannerTrajectory();
        for (int i = 0; i < trajectories.length; i++) {
            finalTrajectory.concatenate(trajectories[i]);
        }
        return finalTrajectory;
    }
    public static CommandBase commandifyTrajectory(Trajectory trajectory, DriveSubsystem drivetrain) {
        //making classes for the autobuilder cause organization is good i think idk
        PIDController pidController = new PIDController(
            AutoConstants.kTurnP,
            AutoConstants.kTurnI,
            AutoConstants.kTurnD);
        SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter);
        
        return new RamseteCommand(
            trajectory,
            drivetrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            motorFeedforward,
            DriveConstants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            pidController,
            pidController,
            drivetrain::tankDriveVolts,
            drivetrain);
    }
    public static CommandBase followTrajectoryWithEvents(PathPlannerTrajectory path, DriveSubsystem drivetrain, String[] markerNames, CommandBase... markerCommands) {
        HashMap<String, Command> eventHashMap = new HashMap<>();

        for (int i = 0; i < markerCommands.length; i++) {
            try {
                eventHashMap.put(markerNames[i], markerCommands[i]);
            } catch (Exception e) {
                System.out.println("ERROR: It appears that the amount of markerNames and markerCommands do not match! Check calls of the method followTrajectoryWithEvents!");
            }
        }

        return new FollowPathWithEvents(
            commandifyTrajectory(path, drivetrain), 
            path.getMarkers(), 
            eventHashMap);
    }
}