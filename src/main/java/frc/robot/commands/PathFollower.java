package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PathFollower extends CommandBase {
    private DriveSubsystem drivetrain;
    private PathPlannerTrajectory path;
    private boolean firstPath;

    private PathPlannerTrajectory pathPlannerTrajectory;
    private CommandBase pathCommand;

    public PathFollower(DriveSubsystem drivetrain, String path, PathConstraints constraints, boolean firstPath) {
        this.drivetrain = drivetrain;
        this.firstPath = firstPath;

        pathPlannerTrajectory = PathPlanner.loadPath(path, constraints);

        addRequirements(drivetrain);
    }
    public PathFollower(DriveSubsystem drivetrain, PathPlannerTrajectory path, boolean firstPath) {
        this.drivetrain = drivetrain;
        this.firstPath = firstPath;

        pathPlannerTrajectory = path;
        
        addRequirements(drivetrain);

        // Unhandled exception: java.lang.NullPointerException: Cannot invoke "com.pathplanner.lib.PathPlannerTrajectory.sample(double)" because "this.transformedTrajectory" is null
    }

    @Override
    public void initialize() {
        if(firstPath)
            drivetrain.resetOdometry(path.getInitialHolonomicPose());
        
        pathCommand = new PPRamseteCommand(
            pathPlannerTrajectory, 
            drivetrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(AutoConstants.kTurnP, 0, 0),
            new PIDController(AutoConstants.kTurnP, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts, // Voltage biconsumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drivetrain // Requires this drive subsystem   
        ); 
    }

    @Override
    public void execute() {
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}
