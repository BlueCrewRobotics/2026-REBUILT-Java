package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bluecrew.pathplanner.CustomAutoBuilder;
import frc.robot.commands.RumbleControllerWhenDriving;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController auxDriver = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /*Aux Controls */

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver.getHID(), XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver.getHID(), XboxController.Button.kLeftBumper.value);

    private final Trigger cancelRotateToAngle = new Trigger(() -> (driver.getRightX() > 0.1 || driver.getRightX() < -0.1));

    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();

    // Sendable Choosers
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Integer> numOfAutoActions;
    private List<SendableChooser<Command>> selectedPathActions = new ArrayList<>();
    private List<SendableChooser<Command>> selectedNoteActions = new ArrayList<>();
    private boolean hasSetupAutoChoosers = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(
                swerveSubsystem.run(() -> swerveSubsystem.teleopDriveSwerve(
                        driver::getLeftY,
                        driver::getLeftX,
                        driver::getRightX,
                        () -> driver.leftBumper().getAsBoolean()
                ))
        );

        // Configure the button bindings
        configureButtonBindings();
    
        // PathPlanner command templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("Hello"));
        NamedCommands.registerCommand("Score Coral LMID", wristSubsystem.wristToLMID().andThen(elevatorSubsystem.L3Reef()));
        NamedCommands.registerCommand("L1 Score", elevatorSubsystem.elevatorToIntakeAuto().andThen(wristSubsystem.wristToL1Auto()));
        NamedCommands.registerCommand("L2 Score", wristSubsystem.wristToLMIDAuto().andThen(elevatorSubsystem.elevatorToL2Auto()));
        NamedCommands.registerCommand("Shoot Coral", wristSubsystem.wristToLMID().andThen(intakeSubsystem.intakeAlgae()).withTimeout(1).andThen(intakeSubsystem.stopIntake()));
        NamedCommands.registerCommand("Get Coral", elevatorSubsystem.returnHome().andThen(wristSubsystem.wristToIntake()));
        NamedCommands.registerCommand("Intake Coral", intakeSubsystem.intakeCoral().onlyWhile(() -> intakeSubsystem.coralInIntake()));
        NamedCommands.registerCommand("Dislodge Shooter", elevatorSubsystem.spinMotor(.2).withTimeout(1).andThen(elevatorSubsystem.stopMotor()));
        NamedCommands.registerCommand("L4 Score", elevatorSubsystem.elevatorToL4Auto().andThen(wristSubsystem.wristToL4Auto()));
        NamedCommands.registerCommand("Home", elevatorSubsystem.returnHome().andThen(wristSubsystem.wristToIntake()));
        NamedCommands.registerCommand("Reset Elevator Position", elevatorSubsystem.runOnce(() -> elevatorSubsystem.resetMotorEncoderToAbsolute()));

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
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
        // driver.povCenter().onFalse(swerveSubsystem.run(() -> swerveSubsystem.teleopDriveSwerve(
        //         () -> driver.getRawAxis(translationAxis),
        //         () -> driver.getRawAxis(strafeAxis),
        //         () -> swerveSubsystem.rotationPercentageFromTargetAngle(Rotation2d.fromDegrees(driver.getHID().getPOV())),
        //         robotCentric
        //         )));
        cancelRotateToAngle.onTrue(new InstantCommand(swerveSubsystem::cancelCurrentCommand));

        driver.leftStick().toggleOnTrue(new RumbleControllerWhenDriving(driver));

        // driver.rightBumper().whileTrue(swerveSubsystem.run(()->swerveSubsystem.teleopSlowTurnDriveSwerve(
        //         driver::getLeftY,
        //         driver::getLeftX,
        //         driver::getRightX,
        //         () -> driver.leftBumper().getAsBoolean()
        //     ))
        // );

        driver.x().whileTrue(swerveSubsystem.teleopDriveSwerveAndRotateToAngleCommand(
                driver::getLeftY,
                driver::getLeftX,
                swerveSubsystem.getTargetYaw(),
                () -> driver.leftBumper().getAsBoolean()
        ));

        // driver.rightStick().toggleOnTrue(swerveSubsystem.run(() -> swerveSubsystem.teleopDriveSwerve(
        //         () -> driver.getRawAxis(translationAxis),
        //         () -> driver.getRawAxis(strafeAxis),
        //         () -> swerveSubsystem.rotationPercentageFromTargetAngle(swerveSubsystem.getAngleToPose(new Translation2d(0, 0))),
        //         robotCentric
        // )));

        driver.rightTrigger().whileTrue(swerveSubsystem.halveRotationSpeed());

        elevatorSubsystem.setDefaultCommand(elevatorSubsystem.run(()->elevatorSubsystem.driveByJoystick(auxDriver::getRightY)));
        wristSubsystem.setDefaultCommand(wristSubsystem.run(()->wristSubsystem.spinByJoystick(auxDriver::getLeftY)));
        
        //intake/ extake controls
        auxDriver.rightBumper().whileTrue(intakeSubsystem.intakeCoral());
        auxDriver.rightBumper().onFalse(intakeSubsystem.stopIntake());
        auxDriver.leftBumper().whileTrue(intakeSubsystem.intakeAlgae());
        auxDriver.leftTrigger().onTrue(intakeSubsystem.stopIntake());
        
        //elevator controls
        auxDriver.povUp().onTrue(elevatorSubsystem.intakeCoral());
        auxDriver.povDown().onTrue(elevatorSubsystem.returnHome());
        auxDriver.povLeft().onTrue(wristSubsystem.wristToLMID().andThen(elevatorSubsystem.L2Reef()));
        auxDriver.povRight().onTrue(wristSubsystem.wristToLMID().andThen(elevatorSubsystem.L3Reef()));
        auxDriver.rightStick().toggleOnTrue(elevatorSubsystem.linearActuatorOut());
        auxDriver.leftStick().toggleOnTrue(elevatorSubsystem.linearActuatorIn());

        //wrist controls
        driver.rightBumper().onTrue(swerveSubsystem.invertDriver());
        auxDriver.x().onTrue(wristSubsystem.wristToIntake());
        auxDriver.a().onTrue(wristSubsystem.wristToL4());
        auxDriver.b().onTrue(wristSubsystem.wristToLMID());
        auxDriver.y().onTrue(wristSubsystem.wristToL4().andThen(elevatorSubsystem.L4Reef()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected(); 
        // Load the path we want to pathfind to and follow
PathPlannerPath path = PathPlannerPath.fromPathFile("B Coarl Station R");

// Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints);
        //return new PathPlannerAuto("New Auto");

        // Command[] autoCommands = new Command[numOfAutoActions.getSelected()*2];

        // for (int i = 0; i < (autoCommands.length/2); i++) {
        //     autoCommands[(i*2)] = selectedPathActions.get(i).getSelected();
        //     autoCommands[(i*2)+1] = selectedNoteActions.get(i).getSelected();
        // }

        // return new SequentialCommandGroup(autoCommands);
    }

    /**
     * Creates all the {@link SendableChooser} for Autonomous
     */
    public void setupAutoChoosers() {
        if(!hasSetupAutoChoosers) {
            for (int i = 0; i < 5; i++) {
                // Sendable Choosers from Custom Pathplanner AutoBuilder
                SendableChooser<Command> pathAction = CustomAutoBuilder.buildAutoChooserFromAutosInPPFolder("Path Actions");
                SendableChooser<Command> noteAction = CustomAutoBuilder.buildAutoChooserFromAutosInPPFolder("Note Actions");


                // Sendable Choosers for testing purposes only
                selectedPathActions.add(pathAction);
                selectedNoteActions.add(noteAction);

//            // Set Default selections
//            selectedPathActions.get(i).setDefaultOption("Path Action 1", Commands.print("Command Path Action 1"));
//            selectedNoteActions.get(i).setDefaultOption("Note Action 1", Commands.print("Command Note Action 1"));
//
//            for (int j = 2; j <= 5; j++) {
//                selectedPathActions.get(i).addOption("Path Action " + j, Commands.print("Command Path Action " + j));
//                selectedNoteActions.get(i).addOption("Note Action " + j, Commands.print("Command Note Action " + j));
//            }

                // Send Choosers to the dashboard
                SmartDashboard.putData("Path Action " + (i + 1), selectedPathActions.get(i));
                SmartDashboard.putData("Note Action " + (i + 1), selectedNoteActions.get(i));
            }
            hasSetupAutoChoosers = true;
        }
    }
}