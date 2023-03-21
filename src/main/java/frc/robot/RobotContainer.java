// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

//import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoTrajectory;
import frc.robot.commands.Autos;
import frc.robot.commands.TurnInPlace;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final PneumaticsSubsystem pneumatics = new PneumaticsSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger xTrigger = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    Trigger yTrigger = new JoystickButton(m_driverController, XboxController.Button.kY.value);

    xTrigger.onTrue(Commands.run(pneumatics::extendPiston, pneumatics));
    yTrigger.onTrue(Commands.run(pneumatics::reversePiston, pneumatics));

    if(m_driverController.getAButtonPressed()) {
      new TurnInPlace(m_robotDrive, 180).execute(); 
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 
  public Command getAutonomousCommand() {
    //   AutoTrajectory autoTrajectory = new AutoTrajectory(m_robotDrive);

    //   return autoTrajectory.DoAutoTrajectory(m_robotDrive);
    return m_robotDrive.followTrajectoryCommand(PathPlanner.loadPath("Jonas", new PathConstraints(0.1, 0.05)));
    //new RunCommand(() -> {System.out.println("HALp");}, m_robotDrive);
    //m_robotDrive.followTrajectoryCommand(PathPlanner.loadPath("New Path", new PathConstraints(4, 3)));

  }
}
