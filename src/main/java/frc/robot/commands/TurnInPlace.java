package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TurnConstants;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;

public final class TurnInPlace extends CommandBase {
    private DriveSubsystem drive;

    private final AHRS navX = new AHRS();
    private PIDController pid = new PIDController(TurnConstants.kTurnP, TurnConstants.kTurnI, TurnConstants.kTurnD);

    private double setpoint;
    private double angle;

    public TurnInPlace(DriveSubsystem drive, double angle) {
        drive = this.drive;
        angle = this.angle;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(5);
    setpoint = (navX.getAngle() + angle);
    pid.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.tankDriveVolts(pid.calculate(navX.getAngle(), setpoint), -pid.calculate(navX.getAngle(), setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pid.atSetpoint()) {
        return true;
    }
    return false;
  }

}
