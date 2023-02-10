package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceOnChargingStation extends CommandBase {

    public CommandBase DoBalanceOnChargingStation(DriveSubsystem m_robotDrive) {
        // Dumbass way to do this not using PID, probably won't work
        if (Math.abs(m_robotDrive.getPitch()) > 7.5) {
            double pitchAngleRadians = m_robotDrive.getPitch() * (Math.PI / 180.0);
            double xAxisRate = Math.sin(pitchAngleRadians) * -1;
            return Commands.runOnce(() -> m_robotDrive.arcadeDrive(xAxisRate, 0), m_robotDrive);
        }

        else {
            return Commands.runOnce(() -> m_robotDrive.tankDriveVolts(0, 0), m_robotDrive);
        }
    }
}