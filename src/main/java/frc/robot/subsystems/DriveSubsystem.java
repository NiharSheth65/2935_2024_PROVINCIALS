// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.StatusVariables;

public class DriveSubsystem extends SubsystemBase {

  // CREATING A CONTAINER FOR PRINTING DRIVE THINGS TO DASHBOARD 
  private final String DRIVE_PREFIX = "SmartDashboard/Drive"; 
  private final String GRYO_PREFIX = "SmartDashboard/Gyro"; 

  // CREATING EACH MOTOR OBJECT 
  private CANSparkMax leftMotorFront = new CANSparkMax(DriveConstants.leftMotorFrontID, MotorType.kBrushless);
  private CANSparkMax leftMotorBack = new CANSparkMax(DriveConstants.leftMotorBackID, MotorType.kBrushless);
  private CANSparkMax rightMotorFront = new CANSparkMax(DriveConstants.rightMotorFrontID, MotorType.kBrushless);
  private CANSparkMax rightMotorBack = new CANSparkMax(DriveConstants.rightMotorBackID, MotorType.kBrushless);

  // CREATING EACH ENCODER OBJECT 

  private RelativeEncoder rightEncoderFront = rightMotorFront.getEncoder(); 
  private RelativeEncoder rightEncoderBack = rightMotorBack.getEncoder(); 

  private RelativeEncoder leftEncoderFront = leftMotorFront.getEncoder(); 
  private RelativeEncoder leftEncoderBack = leftMotorBack.getEncoder(); 

  // CREATING DIFFERENTIAL DRIVE 
  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorFront, rightMotorFront); 

  // CREATE GYRO OBJECT 
  private AHRS navx; 

  // initial reference heading 

  private double referenceHeading; 
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // RESTORING SETTINGS ON ALL MOTOR AND MOTOR CONTROLLER 
    leftMotorFront.restoreFactoryDefaults(); 
    leftMotorBack.restoreFactoryDefaults(); 
    rightMotorFront.restoreFactoryDefaults(); 
    rightMotorBack.restoreFactoryDefaults(); 

    // INVERT ONE SIDE 
    rightMotorFront.setInverted(false);
    leftMotorFront.setInverted(true);

    // SETTING BACK MOTORS TO FOLLOW FRONT 
    rightMotorBack.follow(rightMotorFront); 
    leftMotorBack.follow(leftMotorFront); 

    // ZERO ENCODERS 
    rightEncoderFront.setPosition(0); 
    rightEncoderBack.setPosition(0); 
    leftEncoderFront.setPosition(0); 
    leftEncoderBack.setPosition(0); 


    // GRYO RESET 
    navx = new AHRS(SPI.Port.kMXP); 
    navx.reset();
    navx.zeroYaw(); 

    // BURN FLASH TO SPARK MAX 
    leftMotorBack.burnFlash(); 
    leftMotorFront.burnFlash(); 
    rightMotorFront.burnFlash(); 
    rightMotorBack.burnFlash(); 

    // get the reference heading 
    referenceHeading = getYaw(); 
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Alliance Number", StatusVariables.allianceNumber); 

    // PRINT ENCODER POSITION
    SmartDashboard.putNumber(DRIVE_PREFIX + "left encoder", getLeftEncoderPosition()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "right encoder", getRightEncoderPosition());
    
    // PRINT WHEEL VELOCITY   
    SmartDashboard.putNumber(DRIVE_PREFIX + "left velocity", getLeftEncoderVelocity()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "right velocity", getRightEncoderVelocity()); 

    // PRINT INCHES TRAVELLED BY WHEELS 
    SmartDashboard.putNumber(DRIVE_PREFIX + "left wheel inches", getLeftEncoderToInches()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "right wheel inches", getRightEncoderToInches()); 

    // PRINT MOTOR CURRENT USAGE 
    SmartDashboard.putNumber(DRIVE_PREFIX + "left front wheel current output", getLeftEncoderFrontCurrent()); 
    SmartDashboard.putNumber(DRIVE_PREFIX + "right front wheel current output", getRightEncoderFrontCurrent()); 
  
    // PRINT GRYO VALUES 
    SmartDashboard.putNumber(GRYO_PREFIX + "yaw", getYaw()); 
    SmartDashboard.putNumber(GRYO_PREFIX + "pitch", getPitch()); 
    SmartDashboard.putNumber(GRYO_PREFIX + "roll", getRoll()); 
    SmartDashboard.putNumber(GRYO_PREFIX + "RELATIVE", getAngle0to360()); 
  }

  // SETS ALL MOTOR TO BRAKE 
  public void setBrakeMode(){
    leftMotorBack.setIdleMode(IdleMode.kBrake); 
    leftMotorFront.setIdleMode(IdleMode.kBrake); 
    rightMotorFront.setIdleMode(IdleMode.kBrake); 
    rightMotorBack.setIdleMode(IdleMode.kBrake); 
  }

  // SETS ALL MOTOR TO COAST  
  public void setCoastMode(){
    leftMotorBack.setIdleMode(IdleMode.kCoast); 
    leftMotorFront.setIdleMode(IdleMode.kCoast); 
    rightMotorBack.setIdleMode(IdleMode.kCoast); 
    rightMotorFront.setIdleMode(IdleMode.kCoast); 
  }

  // RESET ENCODRS 
  public void resetEncoders(){
    rightEncoderFront.setPosition(0); 
    leftEncoderFront.setPosition(0); 
  }

  // GET LEFT ENCODER 
  public double getLeftEncoderPosition(){
    return leftEncoderFront.getPosition(); 
  }

  // GET RIGHT ENCODER
  public double getRightEncoderPosition(){
    return rightEncoderFront.getPosition(); 
  }

  // GET LEFT FRONT VELOCITY 
  public double getLeftEncoderVelocity(){
    return leftEncoderFront.getVelocity(); 
  }

  // GET LEFT BACK VELOCITY 
  public double getLeftEncoderBackVelocity(){
    return leftEncoderBack.getVelocity(); 
  }

  // GET RIGHT FRONT VELOCITY 
  public double getRightEncoderVelocity(){
    return rightEncoderFront.getVelocity();  
  }

  // GET RIGHT BACK VELOCITY 
  public double getRightEncoderBackVelocity(){
    return rightEncoderBack.getVelocity(); 
  }

  // GET RIGHT FRONT ENCODER AND CONVERT TO INCHES 
  public double getRightEncoderToInches(){
    return getRightEncoderPosition() * DriveConstants.revToInch; 
  }

  // GET LEFT FRONT ENCODER AND CONVERT TO INCHES 
  public double getLeftEncoderToInches(){
    return getLeftEncoderPosition() * DriveConstants.revToInch; 
  }

  public double getLeftEncoderFrontCurrent(){
    return leftMotorFront.getOutputCurrent(); 
  }

  public double getLeftEncoderBackCurrent(){
    return leftMotorBack.getOutputCurrent(); 
  }

  public double getRightEncoderFrontCurrent(){
    return rightMotorFront.getOutputCurrent(); 
  }

  public double getRightEncoderBackCurrent(){
    return rightMotorBack.getOutputCurrent(); 
  }

  

  // GET AVERAGE ENCODER VALUES BETWEEN RIGHT AND LEFT 
  public double getAverageEncoderDistanceInInches(){
    return (getLeftEncoderToInches() + getRightEncoderToInches())/2; 
  }

  // GET NAVX ROLL 
  public double getRoll(){
    return navx.getRoll(); 
  }

  // GET NAVX PITCH 
  public double getPitch(){
    return navx.getPitch(); 
  }

  // GET NAVX YAW 
  public double getYaw(){
    return navx.getYaw(); 
  }

  // NAVX COMPASS 
  public double getReferenceHeading(){
    return referenceHeading; 
  }

  // GET ANGLE FROM 0 TO 360 INSTEAD OF -180 TO 180 
  public double getAngle0to360(){
   
    double angle = 0; 

    if(getYaw() < 180 && getYaw() > 0){
      angle = getYaw(); 
    }

    else if(getYaw() < 0){
      angle  = 180 + (getYaw() + 180); 
    }

    else if(getYaw() > 395.5){
      angle = 0; 
    }


    return angle; 
  }


  // GET HEADING ERROR BETWEEN DESIRED AND CURRENT HEADING 
  public double getHeadingError(double desiredHeading){

    double error; 

    if(desiredHeading > getAngle0to360() + 180){
      error = (desiredHeading - getAngle0to360()) - 360; 
    }

    else if(desiredHeading < getAngle0to360() - 180){
      error = (desiredHeading - getAngle0to360()) + 360;  
    }

    else{
      error = desiredHeading - getAngle0to360(); 
    }

    return error; 
  }

  // ZERO NAVX 
  public void zeroHeading(){
    navx.reset(); 
    navx.zeroYaw(); 
  }


  // SET PID VALUES ON LEFT SIDE 
  public void setLeftPIDF(double p, double i, double d, double f){
    leftMotorFront.getPIDController().setP(p); 
    leftMotorFront.getPIDController().setI(i); 
    leftMotorFront.getPIDController().setD(d); 
    leftMotorFront.getPIDController().setFF(f); 
  }

  // SET PID VALUES ON RIGHT SIDE 
  public void setRightPIDF(double p, double i, double d, double f){
    rightMotorFront.getPIDController().setP(p); 
    rightMotorFront.getPIDController().setI(i); 
    rightMotorFront.getPIDController().setD(d); 
    rightMotorFront.getPIDController().setFF(f);
  }

  // SET POWER CONSTRAINS ON LEFT SIDE 
  public void leftOutPutConstraints(double min, double max){
    leftMotorFront.getPIDController().setOutputRange(min, max); 
  }

  // SET POWER CONSTRAINTS ON RIGHT SIDE 
  public void setRightOutPutConstraints(double min, double max){
    rightMotorFront.getPIDController().setOutputRange(min, max); 
  }

  // CONTROL RIGHT IN VELOCITY MODE 
  public void setRightVelocityMode(){
    rightMotorFront.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  // CONTROL LEFT IN VELOCITY 
  public void setLeftVelocityMode(){
    leftMotorFront.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  // CONTROL RIGHT IN POWER MODE 
  public void setRightPowerMode(){
    rightMotorFront.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  // SET LEFT SIDE TO POWER MODE 
  public void setLeftPowerMode(){
    leftMotorFront.getPIDController().setReference(0, ControlType.kVoltage);
  }

  // SET VELOCITY OF LEFT AND RIGHT 
  public void setVelocity(double leftVelocity, double rightVelocity){
    leftMotorFront.getPIDController().setReference(leftVelocity, ControlType.kVelocity); 
    rightMotorFront.getPIDController().setReference(rightVelocity, ControlType.kVelocity); 
  }

  // CONTROL LEFT AND RIGHT IN ARCADE MOTOR 
  public void set(double drive, double turn){
    differentialDrive.arcadeDrive(drive, turn);
  }

  // CONTROL LEFT AND RIGHT IN TANK MODE 
  public void setTank(double left, double right){
    differentialDrive.tankDrive(left, right);
  }

  // STOP DRIVE TRAIN 
  public void stop(){
    differentialDrive.stopMotor();
  }

}
