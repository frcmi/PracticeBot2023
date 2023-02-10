package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;

public class BalanceOnChargingStation extends CommandBase {
    
    protected double startBalancePoint = 7.5;
    protected double endBalancePoint = 3.0;

    public CommandBase DoBalanceOnChargingStation(DriveSubsystem m_robotDrive) {
        // Dumbass way to do this not using PID, probably won't work
        //if (Math.abs(m_robotDrive.getPitch()) > startBalancePoint) {
            double pitchAngleRadians = m_robotDrive.getPitch() * (Math.PI / 180.0);
            double xAxisRate = -1 * Math.sin(pitchAngleRadians);
            return Commands.runOnce(() -> m_robotDrive.arcadeDrive(xAxisRate, 0), m_robotDrive);
        //}

        /* 
        else {
            return Commands.runOnce(() -> m_robotDrive.tankDriveVolts(0, 0), m_robotDrive);
        }
        */
        
    }
}