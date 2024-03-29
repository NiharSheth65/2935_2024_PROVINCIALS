// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class TruckSubsystem extends SubsystemBase {

  private final String TRUCK_PREFIX = "SmartDashboard/Truck";
   
  private DigitalOutput LED; 
  public static final int LED_DIO_CHANNEL = LedConstants.truckLightPort;

  private boolean lightsOn = false; 

  /** Creates a new TruckSubsystem. */
  public TruckSubsystem() {
    LED = new DigitalOutput(LED_DIO_CHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean(TRUCK_PREFIX + "TRUCK STATUS", lightsOn);
  }

  public void setTruckBrightness(double brightness){
    lightsOn = brightness > 0.5; // Example threshold for on/off
    if (lightsOn) {
        // Turn on the LED for a portion of the time based on brightness
        // Adjust the delay to control the perceived brightness
        LED.set(true);
    } else {
        // Turn off the LED
        LED.set(false);
    }
  }
}
