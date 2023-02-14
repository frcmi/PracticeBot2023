package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BalanceSubsystem extends SubsystemBase {
  DriveSubsystem driveSubsystem;
  public boolean toggle = false;

  public BalanceSubsystem(DriveSubsystem drv) {
    driveSubsystem = drv;
  }

  public void setToggle(boolean set) {
    toggle = set;
  }

  @Override
  public void periodic() {
    if (!toggle) 
      return;
    if (Math.abs(driveSubsystem.getPitch()) > 3) {
      Commands.runOnce(() -> DoBalanceOnChargingStation()).schedule();
    }
    else {
      new PrintCommand("angle is less than 3 degrees").schedule();
    }
  }

  public void DoBalanceOnChargingStation() {
    double pitchAngleRadians = driveSubsystem.getPitch() * (Math.PI / 180.0);
    double xAxisRate = Math.sin(pitchAngleRadians);
    Commands.runOnce(() -> driveSubsystem.tankDriveVolts(xAxisRate * 7, xAxisRate * 7), driveSubsystem).schedule();
  }
}