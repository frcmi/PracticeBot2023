// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  // Command used for auto is set at bottom of RobotContainer.java in getAutonomousCommand()

  public static CommandBase spinAuto(DriveSubsystem m_drivetrain) {
    return Commands.repeatingSequence(
        new TurnToAngle(90, m_drivetrain), 
        new TurnToAngle(271, m_drivetrain));
  }

  public static CommandBase goForSeconds(Double seconds, DriveSubsystem m_Drivetrain) {
    return Commands.sequence(
      Commands.runOnce(() -> m_Drivetrain.arcadeDrive(1, 0), m_Drivetrain),
      Commands.waitSeconds(seconds), 
      Commands.runOnce(() -> m_Drivetrain.stop(), m_Drivetrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}