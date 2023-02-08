package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceOnChargingStation extends CommandBase {

    public CommandBase DoBalanceOnChargingStation(DriveSubsystem m_robotDrive) {
        // Dumbass way to do this not using PID, probably won't work
        if (m_robotDrive.getPitch() > 10.0f) {
            return Commands.run(() -> m_robotDrive.tankDriveVolts(-0.1, -0.1), m_robotDrive);
        }

        else if (m_robotDrive.getPitch() < -10.0f) {
            return Commands.run(() -> m_robotDrive.tankDriveVolts(0.1, 0.1), m_robotDrive);
        }

        else {
            return Commands.run(() -> m_robotDrive.tankDriveVolts(0, 0), m_robotDrive);
        }
    }
}