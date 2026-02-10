package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//make the shooter work
public class ShooterSubsystem extends SubsystemBase {
private final TalonFX motor1 = new TalonFX(Constants.popcorn);
private final TalonFX motor2 = new TalonFX(Constants.motor);
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
public void ShooterSubsystem(){
shooterConfig.CurrentLimits.SupplyCurrentLimit=40;
shooterConfig.CurrentLimits.SupplyCurrentLimitEnable=true;
}

}
