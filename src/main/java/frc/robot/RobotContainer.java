// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StatusVariables;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.conveyerConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.ClimbCommands.climbManualSpeedCommand;
import frc.robot.commands.ClimbCommands.climbToPositionCommand;
import frc.robot.commands.CommandBlocks.autoDigestNoteCommand;
import frc.robot.commands.CommandBlocks.autoEatNoteCommand;
import frc.robot.commands.CommandBlocks.autoHuntNoteCommand;
import frc.robot.commands.CommandBlocks.autoRevUpCommand;
import frc.robot.commands.ConveyerCommands.conveyCommand;
import frc.robot.commands.ConveyerCommands.conveyerSetSpeedCommand;
import frc.robot.commands.DriveCommands.DefaultDriveCommand;
import frc.robot.commands.DriveCommands.DriveForwardSetDistance;
import frc.robot.commands.IntakeCommand.IntakePowerCommand;
import frc.robot.commands.LedCommands.ledCommand;
import frc.robot.commands.PhotonCommands.photonAlignToTagWithID;
import frc.robot.commands.ShooterCommands.setShooterSpeedCommand;
import frc.robot.commands.ShooterCommands.setShooterVelocityCommand;
import frc.robot.commands.TruckLightCommands.truckCommand;
import frc.robot.commands.VisionCommands.VisionDriveToTargetCommand;
import frc.robot.commands.VisionCommands.VisionTurnToTargetCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TruckSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import org.photonvision.proto.Photon;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(); 
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ConveyerSubsystem m_ConveyerSubsystem = new ConveyerSubsystem();  
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem(); 
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(); 
  private final LightSubsystem m_LightSubsystem = new LightSubsystem(); 
  private final TruckSubsystem m_TruckSubsystem = new TruckSubsystem(); 
  private final PhotonSubsystem m_PhotonSubsystem = new PhotonSubsystem(); 

  private final Joystick joystick = new Joystick(OperatorConstants.primaryControllerPort); 
  private final Joystick joystickSecondary = new Joystick(OperatorConstants.secondaryControllerPort); 

  private final JoystickButton BUTTON_A_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_A_PORT); 
  private final JoystickButton BUTTON_B_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_Y_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_Y_PORT); 
  private final JoystickButton BUTTON_X_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_X_PORT); 
  
  private final JoystickButton BUTTON_RB_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_LB_PORT); 

  // secondary controller 
  private final JoystickButton BUTTON_A_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_A_PORT); 
  private final JoystickButton BUTTON_B_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_Y_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_Y_PORT); 
  private final JoystickButton BUTTON_X_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_X_PORT); 
  
  private final JoystickButton BUTTON_RB_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_LB_PORT); 

  private final CommandGenericHID controllerPrimary = new CommandGenericHID(OperatorConstants.primaryControllerPort);  
  private final CommandGenericHID controllerSecondary = new CommandGenericHID(OperatorConstants.secondaryControllerPort);  


  SendableChooser<Command> m_autoChooser = new SendableChooser<>(); 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    defaultCommands(); 
  }

  private void defaultCommands(){
    m_DriveSubsystem.setDefaultCommand(new DefaultDriveCommand(m_DriveSubsystem, joystick));
    m_TruckSubsystem.setDefaultCommand(new truckCommand(m_TruckSubsystem, 1.0));
    m_LightSubsystem.setDefaultCommand(new ledCommand(m_LightSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // DRIVER CONTROLS 

    // CLIMB PID CONTROLS
    BUTTON_A_PRIMARY.toggleOnTrue(
      new climbToPositionCommand(m_ClimbSubsystem, ClimbConstants.climbFullyExtendedPosition, false)
    ); 

    BUTTON_A_PRIMARY.toggleOnFalse(
      new climbToPositionCommand(m_ClimbSubsystem, ClimbConstants.climbFullyRetractedPosition, true)
    ); 

    // ALIGN TO TARGET 
    BUTTON_B_PRIMARY.onTrue(
      new photonAlignToTagWithID(m_PhotonSubsystem, m_DriveSubsystem, 0, 7, false)
    ); 

    BUTTON_B_PRIMARY.onFalse(
      new photonAlignToTagWithID(m_PhotonSubsystem, m_DriveSubsystem, 0, 7, true)
    ); 

    // AUTO INTAKE 
    BUTTON_Y_PRIMARY.onTrue(
      new SequentialCommandGroup(
        new autoHuntNoteCommand(m_DriveSubsystem, m_VisionSubsystem), 
        new autoEatNoteCommand(m_DriveSubsystem, m_ConveyerSubsystem, m_IntakeSubsystem) 

      )
    ); 

    BUTTON_Y_PRIMARY.onFalse(
      new VisionDriveToTargetCommand(m_DriveSubsystem, m_VisionSubsystem, 0, true)
    ); 

    // CLIMB MANUAL CONTROLS 
    controllerPrimary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new climbManualSpeedCommand(m_ClimbSubsystem, 0)); 
    controllerPrimary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(new climbManualSpeedCommand(m_ClimbSubsystem, ClimbConstants.climbManualExtendSpeed));
    
    controllerPrimary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new climbManualSpeedCommand(m_ClimbSubsystem, 0)); 
    controllerPrimary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(new climbManualSpeedCommand(m_ClimbSubsystem, ClimbConstants.climbManualRetractSpeed)); 

    
    // -----------------------------------------------------------------------------------------------------

    // OPERATOR CONTROLLER

    // USE RIGHT TRIGGER TO SHOOT SPEAKER SPEED

    controllerSecondary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new setShooterSpeedCommand(m_ShooterSubsystem, 0, 0)); 
    
    controllerSecondary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(
        new ParallelCommandGroup(
          new autoRevUpCommand(m_ShooterSubsystem, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed)
        )
    ); 

    // USE LEFT TRIGGER TO SHOOT AMP SPEED

    controllerSecondary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new setShooterSpeedCommand(m_ShooterSubsystem, 0, 0)); 
    
    controllerSecondary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(
        new ParallelCommandGroup(
          new autoRevUpCommand(m_ShooterSubsystem, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed)
        )
    ); 

    // RUNS SHOOTER WHEELS INVERTED TO INTAKE FROM SOURCE 
    BUTTON_A_SECONDARY.onTrue(
      new autoRevUpCommand(m_ShooterSubsystem, -ShooterConstants.speakerAutoTopMotorSpeed, -ShooterConstants.speakerBottomMotorSpeed)
    ); 

    BUTTON_A_SECONDARY.onFalse(
      new setShooterSpeedCommand(m_ShooterSubsystem, 0, 0)
    ); 

    // RUN SHOOTER AMP SPEED AND CENTRE NOTE 
    BUTTON_B_SECONDARY.onTrue(
      new SequentialCommandGroup(
        new autoDigestNoteCommand(m_ConveyerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem), 
        new autoRevUpCommand(m_ShooterSubsystem, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed)
      )
    ); 

    BUTTON_B_SECONDARY.onFalse(
      new conveyerSetSpeedCommand(m_ConveyerSubsystem, 0)
    ); 

    // RUN SHOOTER DISTANCE SPEED AND CENTRE NOTE
    BUTTON_X_SECONDARY.onTrue(
      new SequentialCommandGroup(
        new autoDigestNoteCommand(m_ConveyerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem), 
        new autoRevUpCommand(m_ShooterSubsystem, ShooterConstants.distanceTopMotorSpeed, ShooterConstants.distanceBottomMotorSpeed)
      )
    ); 

    BUTTON_X_SECONDARY.onFalse(
      new conveyerSetSpeedCommand(m_ConveyerSubsystem, 0)
    ); 

    // RUN SHOOTER SPEAKER SPEED AND CENTRE NOTE 
    BUTTON_Y_SECONDARY.onTrue(
      new SequentialCommandGroup(
        new autoDigestNoteCommand(m_ConveyerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem), 
        new autoRevUpCommand(m_ShooterSubsystem, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed)
      )
    ); 

    BUTTON_Y_SECONDARY.onFalse(
      new conveyerSetSpeedCommand(m_ConveyerSubsystem, 0)
    ); 


    // RUN INTAKE AND CONVEYER MANUALLY
    BUTTON_LB_SECONDARY.onTrue(
      new ParallelCommandGroup(
        new IntakePowerCommand(m_IntakeSubsystem, IntakeConstants.outtakeSpeed),  
        new conveyCommand(m_ConveyerSubsystem, conveyerConstants.conveyerOutSpeed)
      )
    ); 

    BUTTON_LB_SECONDARY.onFalse(
      new ParallelCommandGroup(
        new IntakePowerCommand(m_IntakeSubsystem, 0), 
        new conveyCommand(m_ConveyerSubsystem, 0)
      )
    );

    // RUN INTAKE AND CONVEYER MANUALLY
    BUTTON_RB_SECONDARY.onTrue(
      new ParallelCommandGroup(
        new IntakePowerCommand(m_IntakeSubsystem, IntakeConstants.intakeSpeed),  
        new conveyCommand(m_ConveyerSubsystem, conveyerConstants.conveyerInSpeed)
      )
    ); 

    BUTTON_RB_SECONDARY.onFalse(
      new ParallelCommandGroup(
        new IntakePowerCommand(m_IntakeSubsystem, 0), 
        new conveyCommand(m_ConveyerSubsystem, 0)
      )
    );

    // RUN SHOOTER SPEAKER SPEED AND CENTRE NOTE 

    
    // RUN SHOOTER DISTANCE SPEED AND CENTRE NOTE 

    // RUN INTAKE AND CONVEYER MANUAL - SPIT NOTE OUT 

  }

  public static void alliance(){
      Optional<Alliance> ally = DriverStation.getAlliance();
      StatusVariables.allianceNumber = -1; 

      if (ally.isPresent()) {
          if (ally.get() == Alliance.Red) {
              StatusVariables.allianceNumber = 0; 
              StatusVariables.targetID = photonVisionConstants.speakerMiddleRedID; 
            }
            if (ally.get() == Alliance.Blue) {
              StatusVariables.allianceNumber = 1; 
              StatusVariables.targetID = photonVisionConstants.speakerMiddleBlueID;
            }
        }
      else {
          StatusVariables.allianceNumber = -1; 
          StatusVariables.targetID = photonVisionConstants.speakerMiddleRedID; 
      }

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return m_autoChooser.getSelected(); 
  }
}
