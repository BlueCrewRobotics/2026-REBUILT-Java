// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionModule;
import frc.robot.subsystems.VisionPipelineRunnable;
import frc.robot.subsystems.VisionPoseEstimator;
public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
//controlers 
    private final CommandXboxController Driver = new CommandXboxController(0);
    private final CommandXboxController auxDriver = new CommandXboxController(1);
//subsystems 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final ClimberSubsystem ClimberSubsystem = new ClimberSubsystem();
    public final VisionModule visionModule = new VisionModule();
/* 
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Integer> numOfAutoActions;
    private List<SendableChooser<Command>> selectedPathActions = new ArrayList<>();
    private List<SendableChooser<Command>> selectedNoteActions = new ArrayList<>();
    private boolean hasSetupAutoChoosers = false;
*/
    public RobotContainer() {
        /* 
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("Hello"));
        //NamedCommands.registerCommand("Score Coral LMID", wristSubsystem.wristToLMID().andThen(elevatorSubsystem.L3Reef()));
       NamedCommands.registerCommand("Center Climb blue", ClimberSubsystem.ClimbUp().withTimeout(15).andThen(ClimberSubsystem.climbDown()));
        NamedCommands.registerCommand("center shoot",shooterSubsystem.Shoot(Constants.SPEED_OF_SHOTER_LEFT_FACE, Constants.SPEED_OF_SHOTER_RIGHT_FACE).withTimeout(2));
        NamedCommands.registerCommand("shoot auto red", shooterSubsystem.Shoot(Constants.SPEED_OF_SHOTER_LEFT_FACE, Constants.SPEED_OF_SHOTER_RIGHT_FACE).withTimeout(2));
        // Chooser for number of actions in auto
        numOfAutoActions = new SendableChooser<>();
        numOfAutoActions.setDefaultOption("0", 0);
        for (int i = 1; i <= 5; i++) {
            numOfAutoActions.addOption("" + i, i);
        }
        SmartDashboard.putData("Number Of Auto Actions", numOfAutoActions);
        // Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        */
        configureBindings();
    }
     
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(Driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        // controler buttons 
        // intake buttons 
        auxDriver.x().whileTrue(intakeSubsystem.intakeOn(-0.8));
        auxDriver.x().onFalse(intakeSubsystem.intakeOff());
        auxDriver.b().onTrue(intakeSubsystem.intakeOn(0.7));
        //arm buttons 
        auxDriver.povRight().onTrue(armSubsystem.armSetPoints(-8));
        auxDriver.povUp().onTrue(armSubsystem.armToNeutralLevel());
        auxDriver.povLeft().onTrue(armSubsystem.ArmIntake());
        auxDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //system clear
        auxDriver.leftTrigger().whileTrue(shooterSubsystem.Shoot(-.7,-.7));
        auxDriver.leftTrigger().onTrue(intakeSubsystem.intakeOn(0.7));
        auxDriver.leftTrigger().onFalse(shooterSubsystem.stopSpin());
        auxDriver.leftTrigger().onFalse(intakeSubsystem.intakeOff());
        // kicker wheel
        auxDriver.leftBumper().onTrue(shooterSubsystem.kick(.1));
        auxDriver.leftBumper().onFalse(shooterSubsystem.KickOff());
        auxDriver.leftBumper().onTrue(shooterSubsystem.kickT(.1));
        auxDriver.leftBumper().onFalse(shooterSubsystem.KickOffT());
        armSubsystem.setDefaultCommand(
                armSubsystem.setArmMotorSpeed(auxDriver.getLeftY() * -0.5)
                );
      // auxDriver.a().onTrue(armSubsystem.ArmIntake());
       //auxDriver.a().onFalse(armSubsystem.armStop());
        //50 percent wimpy 10ft
        //60 is awsome at 10ft
        //70 to much at 10ft
        // shooter button 
        auxDriver.rightTrigger().whileTrue(shooterSubsystem.Shoot(Constants.SPEED_OF_SHOTER_LEFT_FACE, Constants.SPEED_OF_SHOTER_RIGHT_FACE));
        auxDriver.rightTrigger().onFalse(shooterSubsystem.stopSpin());
        //auxDriver.rightTrigger().whileTrue(shooterSubsystem.spinMotor(.75));
        //auxDriver.rightTrigger().onFalse(shooterSubsystem.stopSpin()); 
        //buttton for motor2
        //auxDriver.rightTrigger().whileTrue(shooterSubsystem.spinMotor2(.7));
        //auxDriver.rightTrigger().onFalse(shooterSubsystem.stopSpin2());
       // all climber stuff
       //Driver.povUp().onTrue(ClimberSubsystem.linearActuatorIn());
       //Driver.povDown().onTrue(ClimberSubsystem.linearActuatorOut());
       //Driver.y().onTrue(ClimberSubsystem.ClimbUp());
       //Driver.a().onTrue(ClimberSubsystem.climbDown());
        Driver.rightBumper().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(Driver.getLeftY(), Driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Driver.back().and(Driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Driver.back().and(Driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Driver.start().and(Driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Driver.start().and(Driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        Driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        // Logic: While the target is in range, rumble. When it leaves range, stop.
    new Trigger(VisionPoseEstimator.getInstance()::isAnyCameraInRange)
    .whileTrue(
        Commands.runEnd(
            () -> Driver.getHID().setRumble(RumbleType.kBothRumble, 1.0),
            () -> Driver.getHID().setRumble(RumbleType.kBothRumble, 0.0)
        ).ignoringDisable(true) // Allows you to test this while the robot is disabled!
    );
    }

    public Command getAutonomousCommand() {
        //return autoChooser.getSelected();
        
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            //ShooterSubsystem.runOnce(()-> shooterSubsystem.Shoot(Constants.SPEED_OF_SHOTER_LEFT_FACE, Constants.SPEED_OF_SHOTER_RIGHT_FACE))
            //.withTimeout(5.0),
            // Finally idle for the rest of auton
            //drivetrain.applyRequest(() -> idle) 
        ); 
        
    }
}

