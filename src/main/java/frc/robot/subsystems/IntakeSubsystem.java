package frc.robot.subsystems;
import java.util.function.BooleanSupplier;


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
//this intialised motor and incoder and spark
public class IntakeSubsystem extends SubsystemBase {
private SparkMax intakeMotor = new SparkMax(Constants.intake_motor_id,SparkLowLevel.MotorType.kBrushless);
private SparkClosedLoopController m_pidController = intakeMotor.getClosedLoopController(); 
private SparkRelativeEncoder leftEncoder = (SparkRelativeEncoder) intakeMotor.getEncoder();
    
    
    

public IntakeSubsystem(){

}

}
