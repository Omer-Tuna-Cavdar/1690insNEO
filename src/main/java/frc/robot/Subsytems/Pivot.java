package frc.robot.Subsytems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HelperMerhodes.HelperMethodes;




public class Pivot extends SubsystemBase {
    private static Pivot instance;
    private final DigitalInput irBeamBreak;

    private final CANSparkMax pivotMotor1;
    private final CANSparkMax pivotMotor2;
    private final CANcoder pivotCANCoder;
    private final SparkPIDController pidController;
    private final RelativeEncoder pivotEncoder;
    private double holdPosition;  // Target position to hold
    public Pivot() {
        pivotMotor1 = new CANSparkMax(Constants.PivotConstants.kPivotMotor1CanId, MotorType.kBrushless);
        pivotMotor2 = new CANSparkMax(Constants.PivotConstants.kPivotMotor2CanId, MotorType.kBrushless);
        pivotCANCoder = new CANcoder(Constants.PivotConstants.kPivotCANCoderId);
        irBeamBreak = new DigitalInput(Constants.PivotConstants.kPivotIrBeamBreakPort);
        // Configure motors
        pivotMotor1.restoreFactoryDefaults();
        pivotMotor2.restoreFactoryDefaults();
        pivotMotor1.setIdleMode(IdleMode.kBrake);
        pivotMotor2.setIdleMode(IdleMode.kBrake);
        // Set pivotMotor2 to follow pivotMotor1
        pivotMotor2.follow(pivotMotor1, true); // true for inverted
        // PID Controller setup for pivotMotor1
        pidController = pivotMotor1.getPIDController();
        pivotEncoder = pivotMotor1.getEncoder();
        pidController.setP(Constants.PivotConstants.kP);
        pidController.setI(Constants.PivotConstants.kI);
        pidController.setD(Constants.PivotConstants.kD);
        pidController.setFF(Constants.PivotConstants.kFF);
        pidController.setOutputRange(Constants.PivotConstants.kMinOutput, Constants.PivotConstants.kMaxOutput);

        // Initialize hold position
        holdPosition = getPosition();
    }

    public static synchronized Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    public void setPosition(double position) {
        holdPosition = position; // Update hold position
        pidController.setReference(holdPosition, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return HelperMethodes.CANcoder0_1toDegrees(pivotCANCoder.getAbsolutePosition().getValueAsDouble());
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(getPosition() - position) < Constants.PivotConstants.kPivotErrorMargin;
    }

    // Commands
    public Command rotateToPosition(double position) {
        this.setPosition(position);
        return this.run(() -> setPosition(position)).until(() -> isAtPositionSetpoint(position));
    }

    public Command stow() {
        return this.run(() -> setPosition(Constants.PivotConstants.kStowPosition))
                .until(() -> isAtPositionSetpoint(Constants.PivotConstants.kStowPosition));
    }

    @Override
    public void periodic() {
        // Continuously hold the target position
        pidController.setReference(holdPosition, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("Pivot Position", getPosition());
        Logger.recordOutput("Pivot Position", getPosition());
    }

    public boolean isPivotBeamBroken() {
        return !irBeamBreak.get(); // IR Beam Break is active-low
    }

    public void resetPivotEncoder() {
        pivotEncoder.setPosition(0);
    }
}
