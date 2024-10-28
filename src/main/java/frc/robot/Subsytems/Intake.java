
package frc.robot.Subsytems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax RollerMotor;
    private final DigitalInput irBeamBreak; // irBeamBreak  connected to DIO port 0 
    public Intake() {
        RollerMotor = new CANSparkMax(Constants.IntakeConstants.kShooterRId, MotorType.kBrushless);
        RollerMotor.setInverted(Constants.IntakeConstants.INTAKE_ROLLER_INVERTED);
        RollerMotor.setIdleMode(IdleMode.kCoast);
        RollerMotor.setSmartCurrentLimit(40);
        irBeamBreak = new DigitalInput(Constants.IntakeConstants.kIntakeIrBeamBreakPort);
    }
    // Intake control methods
    public void runIntake(double speed) {
        RollerMotor.set(speed);
    }

    public void stopIntakeRollers() {
        RollerMotor.set(0);
    }

    public double getrollervoltage() {
        return RollerMotor.getBusVoltage();
    }
    public double getrollerRPM(){
        return RollerMotor.getEncoder().getVelocity();
    }
  
    public void periodic() {
        SmartDashboard.putNumber("intake roller voltage", getrollervoltage());
        SmartDashboard.putBoolean("irBeamBreak", isIntakeBeamBroken());
        Logger.recordOutput("Pivot Position", getrollerRPM());
    }

        public boolean isIntakeBeamBroken() {
        return !irBeamBreak.get(); // Assuming bumper switch is active-low
    }
}
