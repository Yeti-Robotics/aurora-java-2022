// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;
  private int r,g,b;

  public enum LEDStripStatus {
    OFF, ON
  }

  public LEDStripStatus stripStatus;

  public LEDSubsystem() {
    ledStrip = new AddressableLED(LEDConstants.ADDRESSABLE_LED);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    stripStatus = LEDStripStatus.ON;
    ledBuffer.getLED(0);
    SmartDashboard.putNumber("r", r);
    SmartDashboard.putNumber("g", g);
    SmartDashboard.putNumber("b", b);
  }
  @Override
  public void periodic() {
    // super.periodic();
    // r = (int) SmartDashboard.getNumber("r", r);
    // g = (int) SmartDashboard.getNumber("g", g);
    // b = (int) SmartDashboard.getNumber("b", b);
    // for (int i = 0; i < ledBuffer.getLength(); i++) {
    //   setRGB(i, r, g, b);
    // }
    // sendData();
  }

  public void setHSV(int i, int hue, int saturation, int value){
      ledBuffer.setHSV(i, hue, saturation, value);
  }
  
  public void setRGB(int i, int red, int green, int blue){
   ledBuffer.setRGB(i, red, green, blue);
  }

  public int getBufferLength(){
    return ledBuffer.getLength();
  }

  public void sendData(){
    ledStrip.setData(ledBuffer);
  }

  public void stopLEDStrip() {
    ledStrip.stop();
    stripStatus = LEDStripStatus.OFF;
  }

  public void startLEDStrip() {
    ledStrip.start();
    stripStatus = LEDStripStatus.ON;
  }

}
