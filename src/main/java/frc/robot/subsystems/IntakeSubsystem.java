package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

public SparkMax motor; //Motor itself
public SparkMaxConfig config; //motor config
public RelativeEncoder encoder; //built in encoder
public void configMotor(){
    motor = new SparkMax(intake_motor_id, MotorType.kBrushless);//you can configure the motor type bewteen kBrushless and kBrushed
    config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake) //The motor mode, kBrake will stop motor immediately otherwise will spin untill stop naturally
        .inverted(false) //configure if the motor inverted
        .follow(0); //you can set the motor followed to another motor, usually used in 4 motor KOP chassis

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //apply settings to the motor
}
}
