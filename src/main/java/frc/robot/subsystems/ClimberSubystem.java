package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ClimberSubystem {
private final TalonFX motor1 = new TalonFX(Constants.CLIMBER_MOTOR_ID);

    private boolean enableSetPosition = true;
    private double elevatorSetPosition;
    private PositionVoltage elevatorPositionVoltage = new PositionVoltage(0);
    private VelocityVoltage elevatorVelocityVoltage = new VelocityVoltage(0);
    private TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    private CommandXboxController driver;
    private Servo elevatorStopper;
    private double elevatorStopperPosition;
    


}
