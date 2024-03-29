// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StatusVariables;
import frc.robot.Constants.conveyerConstants;


public class ConveyerSubsystem extends SubsystemBase {
  private final String CONVEYER_PREFIX = "SmartDashboard/Conveyer"; 

  private CANSparkMax conveyerTopMotor = new CANSparkMax(conveyerConstants.conveyerTopMotorId, MotorType.kBrushless); 
  private CANSparkMax conveyerBottomMotor = new CANSparkMax(conveyerConstants.conveyerBottomMotorId, MotorType.kBrushless); 

  private RelativeEncoder conveyerTopEncoder = conveyerTopMotor.getEncoder(); 
  private RelativeEncoder conveyerBottomEncoder = conveyerBottomMotor.getEncoder(); 

  // zero - lowest 
  // one -> middle 
  // two - highest 
  private DigitalInput conveyerSwitch1 = new DigitalInput(conveyerConstants.switchOnePort); 
  private DigitalInput conveyerSwitch2 = new DigitalInput(conveyerConstants.switchTwoPort); 
  private DigitalInput conveyerSwitch3 = new DigitalInput(conveyerConstants.switchThreePort); 
  
  /** Creates a new ConveyerSubsystem. */
  public ConveyerSubsystem() {
    conveyerTopMotor.restoreFactoryDefaults(); 
    conveyerBottomMotor.restoreFactoryDefaults(); 

    conveyerTopMotor.setInverted(false);
    conveyerBottomMotor.setInverted(true);

    conveyerTopEncoder.setPosition(0);
    conveyerBottomEncoder.setPosition(0);

    conveyerTopMotor.setSmartCurrentLimit(30);
    conveyerBottomMotor.setSmartCurrentLimit(30);

    conveyerTopMotor.burnFlash();
    conveyerBottomMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer bottom encoder", getConveyerBottomMotorPosition()); 
      SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer top encoder", getConveyerTopMotorPosition()); 
     
     // PRINT CURRENT VELOCITY   
     SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer bottom velocity", getConveyerBottomMotorVelocity()); 
     SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer top velocity", getConveyerTopMotorVelocity()); 
 
     // PRINT MOTOR CURRENT USAGE 
     SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer bottom current", getConveyerBottomMotorCurrent()); 
     SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer top current", getConveyerTopMotorCurrent()); 


     SmartDashboard.putBoolean("sensor one", getConveyerSwitchOneValue()); 
     SmartDashboard.putBoolean("sensor two", getConveyerSwitchTwoValue()); 
     SmartDashboard.putBoolean("sensor three", getConveyerSwitchThreeValue()); 

  }

  public double getConveyerTopMotorPosition(){
    return conveyerTopEncoder.getPosition(); 
  }

  public double getConveyerTopMotorVelocity(){
    return conveyerTopEncoder.getVelocity(); 
  }

  public double getConveyerTopMotorCurrent(){
    return conveyerTopMotor.getOutputCurrent(); 
  }

  public double getConveyerBottomMotorPosition(){
    return conveyerBottomEncoder.getPosition(); 
  }

  public double getConveyerBottomMotorVelocity(){
    return conveyerBottomEncoder.getVelocity(); 
  }

  
  public double getConveyerBottomMotorCurrent(){
    return conveyerBottomMotor.getOutputCurrent(); 
  }


  public boolean getConveyerSwitchOneValue(){
    StatusVariables.conveyerSwitchOneStatus = conveyerSwitch1.get();
    return conveyerSwitch1.get(); 
  }

  public boolean getConveyerSwitchTwoValue(){
    StatusVariables.conveyerSwitchTwoStatus = conveyerSwitch2.get();
    return conveyerSwitch2.get(); 
  }

  public boolean getConveyerSwitchThreeValue(){
    StatusVariables.conveyerSwitchThreeStatus = conveyerSwitch3.get();
    return conveyerSwitch3.get(); 
  }

  public void setBrakeMode(){
    conveyerTopMotor.setIdleMode(IdleMode.kBrake); 
    conveyerBottomMotor.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    conveyerTopMotor.setIdleMode(IdleMode.kCoast); 
    conveyerBottomMotor.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    conveyerTopEncoder.setPosition(0);  
    conveyerBottomEncoder.setPosition(0); 
  }

  public void setTopConveyerPIDF(double p, double i, double d, double f){
    conveyerTopMotor.getPIDController().setP(p); 
    conveyerTopMotor.getPIDController().setI(i); 
    conveyerTopMotor.getPIDController().setD(d); 
    conveyerTopMotor.getPIDController().setFF(f); 
  }

  public void setBottomConveyerPIDF(double p, double i, double d, double f){
    conveyerBottomMotor.getPIDController().setP(p); 
    conveyerBottomMotor.getPIDController().setI(i); 
    conveyerBottomMotor.getPIDController().setD(d); 
    conveyerBottomMotor.getPIDController().setFF(f); 
  }

  public void setConveyerOutPutConstraints(double min, double max){
    conveyerTopMotor.getPIDController().setOutputRange(min, max); 
    conveyerBottomMotor.getPIDController().setOutputRange(min, max); 
  }

  public void setConveyerVelocityMode(){
    conveyerTopMotor.getPIDController().setReference(0, ControlType.kVelocity); 
    conveyerBottomMotor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setConveyerPowerMode(){
    conveyerTopMotor.getPIDController().setReference(0, ControlType.kVoltage); 
    conveyerBottomMotor.getPIDController().setReference(0, ControlType.kVoltage); 
  }

  public void setVelocity(double conveyerVelocity){
    conveyerTopMotor.getPIDController().setReference(conveyerVelocity, ControlType.kVelocity); 
    conveyerBottomMotor.getPIDController().setReference(conveyerVelocity, ControlType.kVelocity); 
  }

  public void setConveyer(double conveyerSpeed){
    conveyerTopMotor.set(conveyerSpeed);
    conveyerBottomMotor.set(conveyerSpeed); 
  }

  public void stop(){
    conveyerTopMotor.stopMotor();
    conveyerBottomMotor.stopMotor();
  }
}
