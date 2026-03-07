package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
public class ArmSubsystemTalon {
private final TalonFX motor1 = new TalonFX(Constants.CLIMBER_MOTOR_ID);
    private boolean enableSetPosition = true;
    private double elevatorSetPosition;
    private PositionVoltage ClimberPositionVoltage = new PositionVoltage(0);
    private VelocityVoltage ClimberVelocityVoltage = new VelocityVoltage(0);
    private TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    private CommandXboxController driver;
    private Servo ClimbStopper;
    private double ClimberStopperPosition;
    private double setPos;
    public ArmSubsystemTalon(){
      motor1.clearStickyFaults();
      
      climberConfig.CurrentLimits.SupplyCurrentLimit = 30;
      climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      climberConfig.Slot0.kP = 0.435;
      climberConfig.Slot0.kI = 0.02;
      climberConfig.Slot0.kD = 0.0;
      climberConfig.Slot0.kG = 0.65;
  
      climberConfig.Slot1.kP = 0.17;
      climberConfig.Slot1.kI = 0.006;//0.005;
      climberConfig.Slot1.kD = 0.01;
      climberConfig.Slot1.kG = 0.65;

      climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor1.getConfigurator().apply(climberConfig);

    
    }
}
