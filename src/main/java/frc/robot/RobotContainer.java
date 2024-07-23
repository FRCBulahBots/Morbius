package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Mechanisms.Intake;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final XboxController driver = new XboxController(0);

   /* Driver Controls */
	private final int translationAxis = XboxController.Axis.kLeftY.value;
	private final int strafeAxis = XboxController.Axis.kLeftX.value;
	private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);

    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kB.value);

    private final JoystickButton xAim = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton disablePickup = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton botOriented = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton pickup = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton reversePickup = new JoystickButton(driver, XboxController.Button.kA.value);

    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();

    /* LEDs */
    PWM greenLED = new PWM(0);

    // Auton Selector
    private SendableChooser<Command> autonSelector = new SendableChooser<Command>();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> botOriented.getAsBoolean(),
                () -> dampen.getAsBoolean(),
                () -> 1 //speed multiplier 
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        //Register named commands
        NamedCommands.registerCommand("IntakeOn", s_Intake.On());
        NamedCommands.registerCommand("IntakeOff", s_Intake.Off());

        // Create Pathplanner Autos
        PathPlannerPath DBAP = PathPlannerPath.fromChoreoTrajectory("NewPath");
        Command DBAPCommand = AutoBuilder.followPath(DBAP);
        PathPlannerPath Test = PathPlannerPath.fromChoreoTrajectory("TestPath");
        Command TestCommand = AutoBuilder.followPath(Test);

        // Auton Selector
        autonSelector.addOption("Do Nothing", null);
        autonSelector.addOption("calibrate swerve module FF Constants", Commands.sequence(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward), Commands.waitSeconds(1), s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse), Commands.waitSeconds(1), s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward), Commands.waitSeconds(1), s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
        autonSelector.addOption("Drive Back and Pick Up", DBAPCommand);
        autonSelector.addOption("Test", TestCommand);
        SmartDashboard.putData("Auton Selector", autonSelector);

        //LEDs
        greenLED.setAlwaysHighMode();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));


        //heading lock bindings
        up.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d90)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        left.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d180)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        right.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d0)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        down.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d270)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        
        //xAim.onTrue(new XAim(s_Swerve)); 

        pickup.onTrue(s_Intake.Intake());
        disablePickup.onTrue(s_Intake.Off());
        reversePickup.onTrue(s_Intake.Reverse());
        reversePickup.onFalse(s_Intake.Off());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonSelector.getSelected();
    }
}
