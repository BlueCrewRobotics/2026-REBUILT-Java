package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ArmSubsystem extends SubsystemBase {
private SparkMax ArmMotor = new SparkMax(Constants.ARM_MOTOR,SparkLowLevel.MotorType.kBrushless);

}
