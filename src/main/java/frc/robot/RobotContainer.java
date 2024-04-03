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
import frc.robot.commands.AutoCommandBlocks.autoDigestNoteCommand;
import frc.robot.commands.AutoCommandBlocks.autoEatNoteCommand;
import frc.robot.commands.AutoCommandBlocks.autoHuntNoteCommand;
import frc.robot.commands.AutoCommandBlocks.autoHuntSpecificTag;
import frc.robot.commands.AutoCommandBlocks.autoHuntTag;
import frc.robot.commands.AutoCommandBlocks.autoRevUpCommand;
import frc.robot.commands.AutoCommandBlocks.autoScoreAmp;
import frc.robot.commands.AutoCommandBlocks.autoScoreTrap;
import frc.robot.commands.AutoCommandBlocks.autoShootNoteCommand;
import frc.robot.commands.AutoCommandBlocks.autoSimpleDigestCommand;
import frc.robot.commands.AutoNoteBlocks.note1Block;
import frc.robot.commands.AutoNoteBlocks.note2Block;
import frc.robot.commands.AutoNoteBlocks.note3Block;
import frc.robot.commands.AutoNoteBlocks.note4Block;
import frc.robot.commands.AutoRoutines.AutoBlueRoutines.fourNoteAuto.BluePreloadCentreLeftAndRight;
import frc.robot.commands.AutoRoutines.AutoBlueRoutines.fourNoteAuto.BluePreloadCentreRightAndLeft;
import frc.robot.commands.AutoRoutines.AutoBlueRoutines.oneNoteAuto.BlueShootPreloadAndExitCommunity;
import frc.robot.commands.AutoRoutines.AutoBlueRoutines.threeNoteAuto.BluePreloadCentreAndLeft;
import frc.robot.commands.AutoRoutines.AutoBlueRoutines.threeNoteAuto.BluePreloadCentreAndRight;
import frc.robot.commands.AutoRoutines.AutoBlueRoutines.twoNoteAuto.BluePreloadAndCentre;
import frc.robot.commands.AutoRoutines.AutoBlueRoutines.twoNoteAuto.BluePreloadAndLeft;
import frc.robot.commands.AutoRoutines.AutoBlueRoutines.twoNoteAuto.BluePreloadAndRight;
import frc.robot.commands.AutoRoutines.AutoRedRoutines.fourNoteAuto.RedPreloadCentreLeftAndRight;
import frc.robot.commands.AutoRoutines.AutoRedRoutines.fourNoteAuto.RedPreloadCentreRightAndLeft;
import frc.robot.commands.AutoRoutines.AutoRedRoutines.oneNoteAuto.RedShootPreloadAndExitCommunity;
import frc.robot.commands.AutoRoutines.AutoRedRoutines.threeNoteAuto.RedPreloadCentreAndLeft;
import frc.robot.commands.AutoRoutines.AutoRedRoutines.threeNoteAuto.RedPreloadCentreAndRight;
import frc.robot.commands.AutoRoutines.AutoRedRoutines.twoNoteAuto.RedPreloadAndCentre;
import frc.robot.commands.AutoRoutines.AutoRedRoutines.twoNoteAuto.RedPreloadAndLeft;
import frc.robot.commands.AutoRoutines.AutoRedRoutines.twoNoteAuto.RedPreloadAndRight;
import frc.robot.commands.ClimbCommands.climbManualSpeedCommand;
import frc.robot.commands.ConveyerCommands.conveyCommand;
import frc.robot.commands.ConveyerCommands.conveyerSetSpeedCommand;
import frc.robot.commands.ConveyerCommands.conveyerTillSensorTwoCommand;
import frc.robot.commands.DriveCommands.DefaultDriveCommand;
import frc.robot.commands.DriveCommands.DriveForwardSetDistance;
import frc.robot.commands.DriveCommands.DriveTurnGyroCommand;
import frc.robot.commands.IntakeCommand.IntakePowerCommand;
import frc.robot.commands.LedCommands.ledCommand;
import frc.robot.commands.PhotonCommands.photonAlignToAnyTag;
import frc.robot.commands.PhotonCommands.photonAlignToTagWithID;
import frc.robot.commands.PhotonCommands.photonAlignToTrap;
import frc.robot.commands.PhotonCommands.photonDriveToAnyTag;
import frc.robot.commands.PhotonCommands.photonDriveToTrap;
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

import java.io.ObjectInputFilter.Status;
import java.util.Optional;

import org.photonvision.proto.Photon;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  // ------------------------------------------------------------

  // RED AUTOS

  // RED ONE PIECE 
  private final Command m_redPreload = new note1Block(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, photonVisionConstants.speakerMiddleRedID); 

  private final Command m_redPreloadAndDriveOut = new RedShootPreloadAndExitCommunity(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem); 
  
  // RED TWO PIECE 
  private final Command m_redTwoPieceCentre = new RedPreloadAndCentre(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  private final Command m_redTwoPieceClear = new RedPreloadAndRight(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  private final Command m_redTwoPieceStage = new RedPreloadAndLeft(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 

  // RED THREE PIECE 
  private final Command m_redThreePieceClear = new RedPreloadCentreAndRight(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  private final Command m_redThreePieceStage = new RedPreloadCentreAndLeft(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  
  // RED FOUR PIECE 
  private final Command m_redFourPieceClearFirstStageLast = new RedPreloadCentreRightAndLeft(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem);
  private final Command m_redFourPieceStageFirstClearLast = new RedPreloadCentreLeftAndRight(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem);

  // BLUE AUTOS

  // RED ONE PIECE 
  private final Command m_bluePreload = new BluePreloadAndCentre(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  private final Command m_bluePreloadAndDriveOut = new BlueShootPreloadAndExitCommunity(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem); 
  
  // RED TWO PIECE 
  private final Command m_blueTwoPieceCentre = new BluePreloadAndCentre(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  private final Command m_blueTwoPieceClear = new BluePreloadAndLeft(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  private final Command m_blueTwoPieceStage = new BluePreloadAndRight(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 

  // RED THREE PIECE 
  private final Command m_blueThreePieceClear = new BluePreloadCentreAndLeft(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  private final Command m_blueThreePieceStage = new BluePreloadCentreAndRight(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem); 
  
  // RED FOUR PIECE 
  private final Command m_blueFourPieceClearFirstStageLast = new BluePreloadCentreLeftAndRight(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem);
  private final Command m_blueFourPieceStageFirstClearLast = new BluePreloadCentreRightAndLeft(m_DriveSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem, m_LightSubsystem, m_PhotonSubsystem, m_VisionSubsystem, m_IntakeSubsystem);


  SendableChooser<Command> m_autoChooser = new SendableChooser<>(); 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    // RED AUTOS --------------------------------------------------------------------------------------------------------------
    
    // ONE PIECE 
    m_autoChooser.setDefaultOption("RED PRELOAD", m_redPreload);
    m_autoChooser.addOption("RED PRELOAD AND EXIT", m_redPreloadAndDriveOut);

    // TWO PIECE  
    m_autoChooser.addOption("RED TWO PIECE PRELOAD AND CENTRE", m_redTwoPieceCentre);
    m_autoChooser.addOption("RED TWO PIECE PRELOAD AND STAGE", m_redTwoPieceStage);
    m_autoChooser.addOption("RED TWO PIECE PRELOAD AND CLEAR", m_redTwoPieceClear);

    // THREE PIECE 
    m_autoChooser.addOption("RED THREE PIECE PRELOAD AND CLEAR", m_redThreePieceClear);
    m_autoChooser.addOption("RED THREE PIECE PRELOAD AND STAGE", m_redThreePieceStage);
    
    // FOUR PIECE 
    m_autoChooser.addOption("RED FOUR PIECE PRELOAD AND CLEAR", m_redFourPieceClearFirstStageLast);
    m_autoChooser.addOption("RED FOUR PIECE PRELOAD AND STAGE", m_redFourPieceStageFirstClearLast);

    // BLUE AUTOS --------------------------------------------------------------------------------------------------------------
    
    // ONE PIECE 
    m_autoChooser.setDefaultOption("BLUE PRELOAD", m_bluePreload);
    m_autoChooser.addOption("BLUE PRELOAD AND EXIT", m_bluePreloadAndDriveOut);

    // TWO PIECE  
    m_autoChooser.addOption("BLUE TWO PIECE PRELOAD AND CENTRE", m_blueTwoPieceCentre);
    m_autoChooser.addOption("BLUE TWO PIECE PRELOAD AND STAGE", m_blueTwoPieceStage);
    m_autoChooser.addOption("BLUE TWO PIECE PRELOAD AND CLEAR", m_blueTwoPieceClear);

    // THREE PIECE 
    m_autoChooser.addOption("BLUE THREE PIECE PRELOAD AND CLEAR", m_blueThreePieceClear);
    m_autoChooser.addOption("BLUE THREE PIECE PRELOAD AND STAGE", m_blueThreePieceStage);
    
    // FOUR PIECE 
    m_autoChooser.addOption("BLUE FOUR PIECE PRELOAD AND CLEAR", m_blueFourPieceClearFirstStageLast);
    m_autoChooser.addOption("BLUE FOUR PIECE PRELOAD AND STAGE", m_blueFourPieceStageFirstClearLast);

    
    Shuffleboard.getTab("Autonomous").add(m_autoChooser); 
    
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
      // new climbToPositionCommand(m_ClimbSubsystem, ClimbConstants.climbFullyExtendedPosition, false)
      new autoScoreTrap(m_DriveSubsystem, m_PhotonSubsystem, m_ShooterSubsystem, m_ConveyerSubsystem)   
      // new DriveTurnGyroCommand(m_DriveSubsystem, 50, false)
    ); 

    BUTTON_A_PRIMARY.toggleOnFalse(
      // new climbToPositionCommand(m_ClimbSubsystem, ClimbConstants.climbFullyRetractedPosition, true)
      new photonDriveToTrap(m_PhotonSubsystem, m_DriveSubsystem, photonVisionConstants.photonTrapTargetPitch, true) 
      // new DriveTurnGyroCommand(m_DriveSubsystem, 0, true)
    ); 

    // ALIGN TO TARGET 
    BUTTON_B_PRIMARY.onTrue(
      new ParallelDeadlineGroup(
        new autoHuntSpecificTag(m_DriveSubsystem, m_PhotonSubsystem, photonVisionConstants.allianceTag, 0, photonVisionConstants.speakerMiddleApproachPitch), 
        new autoRevUpCommand(m_ShooterSubsystem, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed)
      ).andThen(new autoShootNoteCommand(m_DriveSubsystem, m_PhotonSubsystem, m_ConveyerSubsystem, photonVisionConstants.allianceTag))
    ); 

    BUTTON_B_PRIMARY.onFalse(
      new photonDriveToAnyTag(m_PhotonSubsystem, m_DriveSubsystem, photonVisionConstants.speakerMiddleApproachPitch, true) 
    ); 


    // AUTONOMOUS AMP SCORING 

    BUTTON_X_PRIMARY.onTrue(
      new autoScoreAmp(m_DriveSubsystem, m_ConveyerSubsystem, m_ShooterSubsystem, m_IntakeSubsystem, m_PhotonSubsystem)
    ); 

    BUTTON_X_PRIMARY.onFalse(
      new photonDriveToAnyTag(m_PhotonSubsystem, m_DriveSubsystem, photonVisionConstants.speakerMiddleApproachPitch, true) 
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

  public void resetEncoders(){
    m_DriveSubsystem.resetEncoders();
    m_DriveSubsystem.zeroHeading();
    m_ShooterSubsystem.resetEncoders(); 
    // m_ClimbSubsystem.resetEncoders();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    resetEncoders();
    return m_autoChooser.getSelected(); 
  }
}
