package org.frc3512.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc3512.robot.commands.DriveCommands;
import org.frc3512.robot.commands.ShootAndMove;
import org.frc3512.robot.subsystems.conveyor.Conveyor;
import org.frc3512.robot.subsystems.conveyor.ConveyorIO;
import org.frc3512.robot.subsystems.conveyor.ConveyorIO_REAL;
import org.frc3512.robot.subsystems.conveyor.ConveyorIO_SIM;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.drive.GyroIO;
import org.frc3512.robot.subsystems.drive.GyroIOPigeon2;
import org.frc3512.robot.subsystems.drive.ModuleIO;
import org.frc3512.robot.subsystems.drive.ModuleIOSim;
import org.frc3512.robot.subsystems.drive.ModuleIOTalonFX;
import org.frc3512.robot.subsystems.drive.TunerConstants;
import org.frc3512.robot.subsystems.flywheels.Flywheel;
import org.frc3512.robot.subsystems.flywheels.FlywheelIO;
import org.frc3512.robot.subsystems.flywheels.FlywheelIO_REAL;
import org.frc3512.robot.subsystems.flywheels.FlywheelIO_SIM;
import org.frc3512.robot.subsystems.intake.Intake;
import org.frc3512.robot.subsystems.intake.IntakeIO;
import org.frc3512.robot.subsystems.intake.IntakeIO_REAL;
import org.frc3512.robot.subsystems.intake.IntakeIO_SIM;
import org.frc3512.robot.subsystems.led.Leds;
import org.frc3512.robot.subsystems.vision.Vision;
import org.frc3512.robot.subsystems.vision.VisionConstants;
import org.frc3512.robot.subsystems.vision.VisionIO;
import org.frc3512.robot.subsystems.vision.VisionIOPhotonVision;
import org.frc3512.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Flywheel flywheel;
  private final Intake intake;
  private final Conveyor conveyor;

  private final Leds leds = Leds.getInstance();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Debug Sticks
  //   private final CommandJoystick debugSimple = new CommandJoystick(5);
  private final CommandJoystick debugComplex = new CommandJoystick(4);

  // Dashboard inputs
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.GeneralConstants.currentMode) {
      case REAL:
        // Real robot, instantiate real hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.frontLeftCamera, VisionConstants.robotToLeft),
                new VisionIOPhotonVision(
                    VisionConstants.frontRightCamera, VisionConstants.robotToRight));
        // new VisionIOPhotonVision(
        //     VisionConstants.rearCamera, VisionConstants.robotToRear));

        flywheel = new Flywheel(new FlywheelIO_REAL());
        intake = new Intake(new IntakeIO_REAL());
        conveyor = new Conveyor(new ConveyorIO_REAL());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.frontLeftCamera, VisionConstants.robotToLeft, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.frontRightCamera,
                    VisionConstants.robotToRight,
                    drive::getPose));

        flywheel = new Flywheel(new FlywheelIO_SIM());
        intake = new Intake(new IntakeIO_SIM());
        conveyor = new Conveyor(new ConveyorIO_SIM());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});
        intake = new Intake(new IntakeIO() {});
        conveyor = new Conveyor(new ConveyorIO() {});

        break;
    }

    NamedCommands.registerCommand("Far Shoot", farShoot());
    NamedCommands.registerCommand("Close Shoot", closeShoot());
    NamedCommands.registerCommand("Intake", intake.setRollerSpeed(0.5));
    NamedCommands.registerCommand("Stop Intake", reset());

    autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the button bindings
    configureButtonBindings();

    // Set up auto routines
    SmartDashboard.putData("Auto Modes", autoChooser);
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when left stick is pressed
    controller.leftStick().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when right stick is pressed
    controller
        .rightStick()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.rightTrigger().onTrue((autoShoot())).onFalse(idle());
    controller.leftTrigger().onTrue(intake()).onFalse(prepShooter());

    controller.b().whileTrue(feed());

    // Complex Debug Binds
    debugComplex.button(1).onTrue(reset());

    debugComplex
        .button(9)
        .onTrue(flywheel.setRPM(4100))
        .onTrue(feed())
        .onFalse(idle())
        .onFalse(flywheel.stop());

    debugComplex.button(10).onTrue(intake.setRollerSpeed(0.8));
    debugComplex.button(11).onTrue(intake.setRollerSpeed(-0.5));
    debugComplex.button(12).onTrue(intake.setRollerSpeed(0.0));
  }

  // Methods
  public Command reset() {
    return Commands.sequence(
        intake.setRollerSpeed(0), conveyor.setHopper(0), conveyor.setFeeder(0), flywheel.stop());
  }

  // For Actual Testing
  public Command autoShoot() {
    return Commands.sequence(
        // Aim Robot and set Flyhweel RPM
        new ShootAndMove(
            drive, flywheel, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
  }

  public Command idle() {
    return Commands.sequence(
        intake.setRollerSpeed(0.15),
        conveyor.setHopper(0),
        conveyor.setFeeder(0),
        flywheel.setRPM(750));
  }

  public Command intake() {
    return Commands.sequence(intake.setRollerSpeed(0.75));
  }

  public Command prepShooter() {
    return Commands.sequence(intake.setRollerSpeed(0.15), flywheel.setRPM(1250));
  }

  // Simple
  public Command feed() {
    return Commands.sequence(conveyor.setHopper(0.0), conveyor.setFeeder(0.5));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // Auto Methods
  public Command farShoot() {
    return Commands.sequence(
        flywheel.setOutput(0.65),
        Commands.waitSeconds(3),
        feed(),
        Commands.waitSeconds(6),
        reset());
  }

  public Command closeShoot() {
    return Commands.sequence(
        flywheel.setOutput(0.55),
        Commands.waitSeconds(3),
        feed(),
        Commands.waitSeconds(6),
        reset());
  }

  public Command intakeDepot() {
    return Commands.sequence();
  }

  // Underglow
  public Command setUnderglow() {
    return Commands.either(
        leds.runPattern(leds.red),
        leds.runPattern(leds.blue),
        () -> DriverStation.getAlliance().get() == Alliance.Red);
  }
}
