// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // OPERATOR CONSTANTS
  public static class OperatorConstants {
    
    // CONTROLLER PORTS 
    public static final int primaryControllerPort = 0; 
    public static final int secondaryControllerPort = 1; 

    // BUTTON MAPPING 
    public static final int BUTTON_A_PORT = 1;
    public static final int BUTTON_B_PORT = 2;
    public static final int BUTTON_X_PORT = 3;
    public static final int BUTTON_Y_PORT = 4;

    public static final int BUTTON_RB_PORT = 6;
    public static final int BUTTON_LB_PORT = 5;

    public static final int BUTTON_START = 8;
    public static final int BUTTON_RIGHT_JOYSTICK_PORT = 9;
    public static final int BUTTON_LEFT_JOYSTICK_PORT = 10;
  
    public static final int driveJoystickAxis = 1; 
    public static final int turnJoystickAxis = 4; 

    public static final int rightTriggerAxis = 3; 
    public static final int leftTriggerAxis = 2; 

    // TRIGGER ACTIVATION THRESHOLD 
    public static final double triggerThreshold = 0.5; 
  }


  // DRIVE TRAIN CONSTANTS 
  public static class DriveConstants{

    // MOTOR CAN IDS 
    public static final int leftMotorFrontID = 1; 
    public static final int leftMotorBackID = 2; 
    public static final int rightMotorFrontID = 3; 
    public static final int rightMotorBackID = 4; 

    // SLEW RATES 
    public static final double driveSlew = 5; 
    public static final double turnSlew = 2; 

    public static final double driveFastSlew = 10; 
    public static final double turnFastSlew = 10; 

    // DRIVE SPEEDS 
    public static final double driveSlowSpeed = 0.85; 
    public static final double driveFastSpeed = 1.0; 

    // TURN SPEEDS 
    public static final double turnSlowSpeed = 0.5; 
    public static final double turnFastSpeed = 0.75; 

    // DEAD BAND 
    public static final double driveDeadBand = 0.1; 
    public static final double turnDeadBand = 0.1; 

    // ENCODER CONVERSIONS 
    public static final double encoderToInchConversion = 1/42 * ((Math.PI * 4) / 5.95);
    
    public static final double inchToRev = (5.95/12.56); 
    public static final double revToInch = (12.56/5.95); 

    // DRIVE SPEED DURING AUTONOMOUS 
    public static final double autoSpeed = 0.70; 

    // DRIVE SET DISTANCE PID 
    public static final double driveDistanceKp = 0.075; 
    public static final double driveDistanceKi = 0.01; 
    public static final double driveDistanceKd = 0.0; 

    // GRYO TURN PID 
    public static final double turnKp = 0.01; 
    public static final double turnKi = 0.0; 
    public static final double turnKd = 0.0; 

    // DRIVE FORWARD INTAKING DISTANCE
    public static final double driveForwardToIntake = 36; 
  }


  // GYRO CONSTANTS 
  public static class GyroConstants{
    
    // GYRO END CONSTRAINS 
    public static final double autoGyroHasTurnedTolerance = 5; 
    public static final double autoGyroTurnTimeOut = 2000; 


    // GYRO COMMAND TURN SPEED 
    public static double gyroTurnMaxSpeed = 0.8; 

    // GYRO TURN PID 
    public static double gryoTurnKp = 0.035; 
    public static double gryoTurnKi = 0.010; 
    public static double gryoTurnKd = 0; 
  }

  // SHOOTER CONSTANTS
  public static class ShooterConstants{

    // MOTOR IDS FOR SHOOTER MOTORS 
    public static final int topShooterMotorId = 6; 
    public static final int bottomShooterMotorId = 5; 

    public static final double shooterFullSpeed = 1.0; 
    public static final double shooterOffSpeed = 0.0; 
    
    // SPEAKER SHOT 
    public static final double speakerTopMotorSpeed = -5700*0.90; 
    public static final double speakerBottomMotorSpeed = 5700*0.90; 

    // TRAP SHOT 
    public static final double trapTopMotorSpeed = -5700*0.80; 
    public static final double trapBottomMotorSpeed = 5700*0.83; 

    // DISTANCE SHOT
    public static final double distanceTopMotorSpeed = -5700*0.95; 
    public static final double distanceBottomMotorSpeed = 5700*0.40; 

    public static final double speakerAutoTopMotorSpeed = -5700*0.75; 
    public static final double speakerAutoBottomMotorSpeed = 5700*0.40;
      

    // AMP SHOT 
    public static final double ampTopMotorSpeed = 5700 * -0.18; 
    public static final double ampBottomMotorSpeed = 5700 * 0.58; 

    public static final double autoCentreTopMotorSpeed = -5700 * 0.50; 
    public static final double autoCentreBottomMotorSpeed = 5700 * 0.50; 


    // SPEED TO KEEP NOTE IN DURING CENTRING 
    public static final double shooterHoldInSpeed = 5700 * -0.1; 

     // TOP SHOOTER MOTOR PID 
     public static double topShooterKp = 6e-5; 
     public static double topShooterKi = 0.0000; 
     public static double topShooterKd = 0; 
     public static double topShooterKIz = 6e-5; 
     public static double topShooterKFf = 0.000015;
 
     // BOTTOM SHOOTER MOTOR  PID
     public static double bottomShooterKp = 6e-5; 
     public static double bottomShooterKi = 0.0000; 
     public static double bottomShooterKd = 0; 
     public static double bottomShooterKIz = 6e-5; 
     public static double bottomShooterKFf = 0.000015;
 
     // SHOOTER MOTOR OUTPUT CONSTRAINTS
     public static double topMax = 1; 
     public static double topMin = -1; 
 
     public static double bottomMax = 1; 
     public static double bottomMin = -1; 
 
     // SHOOTER MOTOR SLEW RATE LIMITERS 
     public static double topSlewLimit = 2; 
     public static double bottomSlewLimit = 2; 
 
     // SHOOTER MOTOR RAMP RATE 
     public static double shooterRampRate = 0.25;
  }


  // INTAKE CONSTANTS 
  public static class IntakeConstants{
    
    // INTAKE MOTOR ID  
    public static final int intakeMotorId = 9;

    // INTAKE MOTOR SPEED
    public static final double intakeSpeed = 0.75; 

    public static final double intakeStopSpeed = 0; 
    
    public static final double outtakeSpeed = -0.45;

    public static final double intakeVelocity = 0.10 * 11000; 
    public static final double outtakeVelocity = -0.10 * 11000; 

    
    public static final double intakeStopVelocity = 0; 

    public static final int intakeHasIntakedThreshhold = 20; 
    
    

  }

  public static class VisionConstants{
    // LIMELIGHT SET UP CONSTANTS 
    public static final double limlightLensHeight = 19.5; 
    public static final double limelightMountAngle = -6; 
    public static final double goalHeight = 0; 

    // LIMELIGHT PIPELINES 
    public static final int ambientPipeline = 0; 

    // TOLERANCES FOR LINING UP TO NOTE 
    public static final double roughAlignmentTolerance = 7.5; 
    public static final double fineAlignmentTolerance = 5.5; 

    // STOPPING BEFORE NOTE DISTANCE 
    public static final double limelightDesiredApproachDistance = 35; 

    // DRIVE FORWARD AND INTAKING AFTER STOPPING 
    public static final double limelightDriveForwardAndIntakeDistance = limelightDesiredApproachDistance + 3;

    // VISION DRIVE TO PITCH 
    public static final double limelightReadToIntakePitch = -15; 
    
    // LIMELIGHT TURN AND DRIVE SPEED LIMITS 
    public static final double limelightTurnSpeedLimit = 0.40; 
    public static final double limelightDriveSpeedLimit = 0.70; 

    // TIME OUTS FOR LIMELIGHT COMMANDS 
    public static final double limelightTargetAcquiredTimeOut = 1000; 
    public static final double limelightTurnTargetingTimeOut= 3000; 
    public static final double limelightCommandTimeout = 4000; 

    // LIMELIGHT GET IN RANGE TOLERANCE 
    public static final double limelightGetInRange = 2; 

    // CALCULATING ADJUSTMENT FOR LIMELIGHT TURNS 
    public static double adjust = 0; 

    // GRYO ANGLE WHEN DRIVE IS COMPLETE 
    public static double driveStartGyroAngle; 
    public static double adjustEndGyroAngle;  

    // VISION PIDS 
    public static double visionDriveKp = 0.25; 
    public static double visionDriveKi = 0.00; 
    public static double visionDriveKd = 0.00; 

    public static double visionTurnKp = 0.04; 
    public static double visionTurnKi = 0.02; 
    public static double visionTurnKd = 0.00;





  }

  public static class conveyerConstants{
      //CONVEYER MOTOR ID  
      public static final int conveyerTopMotorId = 7;
      public static final int conveyerBottomMotorId = 8;

      // CONVEYER INTAKE SPEED
      public static final double conveyerInSpeed = 1.0; 

      // CONVEYER OUTTAKE SPEED 
      public static final double conveyerOutSpeed = -1.0; 

      // SWITCH PORTS 
      public static final int switchOnePort = 0; 
      public static final int switchTwoPort = 1; 
      public static final int switchThreePort = 2; 

      // CONVEYER TIMEOUT DURATION 
      public static final double conveyerCommandTimeOut = 3000; 
  }

  public static class photonVisionConstants{

    // CAMERA NAME 
    public static String cameraName= "wide-angle-camera"; 
    
    // CAMERA HEIGHT AND ANGLE 
    public static double cameraHeight = Units.inchesToMeters(22.5); 
    public static double cameraMountAngle = Units.degreesToRadians(22); 

    // HEIGHTS OF DIFFERENT APRIL TAG TARGETS 
    public static double ampHeight = Units.inchesToMeters(0); 
    public static double speakerHeight = Units.inchesToMeters(57); 
    public static double sourceHeight = Units.inchesToMeters(0); 
    public static double trapHeight = Units.inchesToMeters(0); 
  
    // APPROACH DISTANCES OF DIFFERENT APRIL TAGS 
    public static double ampDistance = Units.inchesToMeters(20); 
    public static double speakerDistance = Units.inchesToMeters(0); 
    public static double sourceDistance = Units.inchesToMeters(0); 
    public static double trapDistance = Units.inchesToMeters(0); 

    // PHOTON DRIVE PID 
    // public static double driveKp = 0.4; 
    public static double driveKp = 0.3; 
    public static double driveKi = 0; 
    public static double driveKd = 0; 

    // PHOTON TURN PID 
    public static double turnKp = 0.03; 
    public static double turnKi = 0.025; 
    public static double turnKd = 0; 
    
    // PHOTON MAX DRIVE AND SPEED CUT OFFS 
    public static double photonMaxTurnSpeed = 0.3; 
    public static double photonMaxDriveSpeed = 0.60; 

    // APRIL TAG IDS 
    public static int sourceLeftBlueID = 1; 
    public static int sourceRightBlueID = 2; 

    public static int speakerMiddleRedID = 4;
    // public static int speakerMiddleRedID = 5;
    public static int speakerOffCentreRedID = 3;

    public static int ampRedID = 5; 
    public static int ampBlueID = 6;

    public static int speakerMiddleBlueID = 7;
    public static int speakerOffCentreBlueID = 8;

    public static int sourceLeftRedID = 9; 
    public static int sourceRightRedID = 10; 

    public static int stageRedLeftID = 11; 
    public static int stageRedRightID = 12; 
    public static int stageRedCentreID = 13; 

    public static int stageBlueLeftID = 14; 
    public static int stageBlueRightID = 15; 
    public static int stageBlueCentreID = 16; 

    public static double speakerMiddleApproachPitch = 0; 
    public static double speakerMiddleAlignYaw = 0; 

    // photon alignment tight tolerance 
    public static double photonTightTolerance = 5; 

    // PHOTON COMMAND TIMEOUTS
    public static double photonTargetAcquiredTimeOut = 2000; 
    public static double photonTurnTargetingTimeOut = 4000; 
    public static double photonDriveTargetingTimeOut= 2500; 

    // PHOTON STAGE VARIABLES 
    public static double photonTrapTargetPitch = 12.5;  
    public static double photonTrapTargetYaw = -6.10; 
    public static double photonTrapDriveSpeed = 0.35; 

    public static int allianceTag = speakerMiddleBlueID; 
  }

  public static class autoConstants{
    
    // DRIVE COMMAND TIMOUT OUTS 
    public static double autoDriveDistanceTimeOut= 3000; 
    public static double autoGyroTurnTimeOut = 3000;  

    // DRIVE TOLERANCE 
    public static double autoDriveDistanceHasReachedTolerance = 2; 

    // TURN TOLERANCE 
    public static double autoGyroHasTurnedTolerance = 5;


    // DRIVE BACK DISTANCES 
    public static double driveBackDistanceFromNote2 = 55; 

    // DISTANCE DRIVEN BY PHOTON VISION
    public static double distanceDrivenDurringPhoton; 
  }

  public static class LedConstants{
    // LED PWM PORT 
    public static final int ledPort = 1; 

    // TRUCK LIGHT DIO PORT 
    public static final int truckLightPort = 3; 
    
    // LED PIXEL LENGTH 
    public static final int ledLength = 60; 
    
    // INDIVIDUAL COLOUR CODES 
    public static final int[] greenColourCode = {0, 255, 0}; 
    public static final int[] blueColourCode = {0, 0, 255}; 
    public static final int[] redColourCode = {255, 0, 0}; 
    public static final int[] orangeColourCode = {255, 25, 0}; 
    public static final int[] whiteColourCode = {255, 125, 50}; 
    public static final int[] vermillionColourCode = {255, 255, 255}; 
    public static final int[] purpleColourCode = {200, 0, 200}; 
    public static final int[] yellowColourCode = {200, 150, 0}; 

  }

  public static class ClimbConstants{
    
    // CLIMB MOTOR ID 
    public static int climbMotorID = 10; 

    // CLIMB PID VALUES 
    public static double climbKp = 0.2; 
    public static double climbKi = 0; 
    public static double climbKd = 0; 

    // CLIMB MAX SPEEDS DURRING MANUAL CONTROL
    public static double climbManualRetractSpeed = 0.5; 
    public static double climbManualExtendSpeed = -0.5; 

    // PID CLIMB MAX SPEED
    public static double climbMaxSpeed = 1.0; 

    // ENCODER POSITIONS FOR FULLY RETRACTED AND EXTENDED
    public static double climbFullyRetractedPosition = 0; 
    public static double climbFullyExtendedPosition = 436; 
  }

  public static class StatusVariables{
    public static boolean targetVelocityReached;
    public static boolean conveyerSwitchOneStatus;  
    public static boolean conveyerSwitchTwoStatus;  
    public static boolean conveyerSwitchThreeStatus;  
    public static boolean revedUpActivated = false; 
    public static int allianceNumber; 
    public static int targetID; 
  }


}
