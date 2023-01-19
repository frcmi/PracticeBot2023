package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class PneumaticsSubsystem extends SubsystemBase {
  Compressor compressor = new Compressor(PneumaticConstants.compressorPort, PneumaticsModuleType.CTREPCM);

  DoubleSolenoid solenoidLeft = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, PneumaticConstants.extendSolenoidPortLeft, PneumaticConstants.retractSolenoidPortLeft);
  DoubleSolenoid solenoidRight = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, PneumaticConstants.extendSolenoidPortRight, PneumaticConstants.retractSolenoidPortRight);

  private boolean toggleState = true; // Set to true so on first toggle will extend
  public PneumaticsSubsystem() {
    compressor.enableDigital();
  }

  public void extendPiston() {
    if (!compressor.getPressureSwitchValue()) {
      solenoidLeft.set(Value.kForward);
      solenoidRight.set(Value.kForward);
    }
  }

  public void stopPiston() {
    solenoidLeft.set(Value.kOff);
    solenoidRight.set(Value.kOff);
  }

  public void reversePiston() {
    solenoidLeft.set(Value.kReverse);
    solenoidRight.set(Value.kReverse);
  }

  public void togglePiston() {
    if (toggleState)
      extendPiston();
    else
      reversePiston();
  }

  public Command toggleCommand() {
    return Commands.run(() -> togglePiston(), this);
  }
}
