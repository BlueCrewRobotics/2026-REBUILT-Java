package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
// CHANGE: Use DutyCycle instead of Voltage for non-Pro
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystemKraken extends SubsystemBase {
    private PositionVoltage ArmSubsystemVoltage = new PositionVoltage(0);
    private final TalonFX armMotor;
    private final CANcoder armCanEncoder;

    // CHANGE: Use MotionMagicDutyCycle for Phoenix Free
    private final MotionMagicDutyCycle m_mmReq = new MotionMagicDutyCycle(0);

    private final double gravityFF = 0.07; 

    public ArmSubsystemKraken() {
        armMotor = new TalonFX(Constants.ARM_MOTOR);
        armCanEncoder = new CANcoder(Constants.ARM_CAN_ENCODER);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        
    

        motorConfig.Slot0.kP = Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kP;
        motorConfig.Slot0.kI = Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kI;
        motorConfig.Slot0.kD = Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kD;
        
        motorConfig.MotionMagic.MotionMagicAcceleration = 40; 
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 80;

        motorConfig.CurrentLimits.StatorCurrentLimit = 25;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Apply Configuration
        armMotor.getConfigurator().apply(motorConfig);

        CANcoderConfiguration canConfig = new CANcoderConfiguration();
     
   canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armCanEncoder.getConfigurator().apply(canConfig);

        // CHANGE: Since we can't use FusedCANcoder without Pro, 
        // we manually seed the motor position to match the CANcoder once at startup.
        armMotor.setPosition(armCanEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public double getArmDegrees() {
        return armCanEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    private double getGravityFF() {
        double radians = Math.toRadians(getArmDegrees() - Constants.ARM_BALANCE_DEGREES);
        return Math.sin(radians) * gravityFF;
    }

    public Command armUp() {
        return new StartEndCommand(
            // DutyCycle setpoints are still in rotations (0 to 1)
            //armMotor.setControl(ArmSubsystemVoltage.withPosition(-10));
           () ->  armMotor.set(-0.1) ,
           () ->  armMotor.set(0) ,
           this
        );
    }

    public Command armDown() {
        return new StartEndCommand(
            () ->  armMotor.set(0.1) ,
           () ->  armMotor.set(0) ,
           this
        );
    }

    public Command armWiggle() {
        return new StartEndCommand(
            () ->  armMotor.set(0.1) ,
           () ->  armMotor.set(0) ,
           this
        );
    }

    public Command stopArm() {
        return new InstantCommand(() -> armMotor.stopMotor());
    }
}