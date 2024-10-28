package frc.robot.Subsytems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private final CANSparkMax ShooterL;
    private final CANSparkMax ShooterR ;
    private final PIDController shooterPidController;
    public Shooter(){
        ShooterL = new CANSparkMax(Constants.ShooterConstans.kShooterLId, MotorType.kBrushless);
        ShooterR = new CANSparkMax(Constants.ShooterConstans.kShooterRId, MotorType.kBrushless);
        shooterPidController = new PIDController(Constants.ShooterConstans.kShooterP, Constants.ShooterConstans.kShooterI, Constants.ShooterConstans.kShooterD);
        shooterPidController.setTolerance(Constants.ShooterConstans.kShooterPositionTolerance, Constants.ShooterConstans.kShooterVelocityTolerance);
        ShooterL.setIdleMode(IdleMode.kCoast);
        ShooterR.setIdleMode(IdleMode.kCoast);
        ShooterL.setSmartCurrentLimit(40);
        ShooterR.setSmartCurrentLimit(40);
        }
       public void runShooter(double RPM) {
        double outputR = shooterPidController.calculate(ShooterR.getEncoder().getPosition(), RPM);
        double outputL = shooterPidController.calculate(ShooterL.getEncoder().getPosition(), RPM);
        MathUtil.clamp(outputR, -0.7, 0.7);
        MathUtil.clamp(outputL, -0.7, 0.7);
        ShooterR.set(outputR);
        ShooterL.set(-outputL);
    }

    public boolean isAtTargetRPM(double targetRPM) {
        double currentRPM = getShooterRPM();
        return Math.abs(currentRPM - targetRPM) <= Constants.ShooterConstans.kShooterVelocityTolerance;
    }

    public void stopShooter() {
        ShooterR.set(0);
        ShooterL.set(0);
    }
    public double getShooterRPM(){
        return ShooterR.getEncoder().getVelocity();
    }
    public double getShooterAMPS(){
        return ShooterR.getOutputCurrent();
    }
    public double getShooterVoltage(){
        return ShooterR.getBusVoltage();
    }
    public void periodic(){
        SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
        SmartDashboard.putNumber("Shooter AMPS", getShooterAMPS());
        SmartDashboard.putNumber("Shooter Voltage", getShooterVoltage());
        Logger.recordOutput("Shooter RPM", getShooterRPM());
    }
    public double getShooterVelocity(){
        return ShooterR.getEncoder().getVelocity();
    }
}