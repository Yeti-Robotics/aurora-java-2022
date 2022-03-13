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
  private AddressableLED leftStrip;
  private AddressableLEDBuffer leftBuffer;
  private AddressableLED rightStrip;
  private AddressableLEDBuffer rightBuffer;

  private int r, g, b;

  public enum LEDStripStatus {
    OFF, ON
  }

  public LEDStripStatus stripStatus;

  public LEDSubsystem() {
    leftStrip = new AddressableLED(LEDConstants.LEFT_ADDRESSABLE_LED);
    leftBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
    rightStrip = new AddressableLED(LEDConstants.RIGHT_ADDRESSABLE_LED);
    rightBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

    leftStrip.setLength(leftBuffer.getLength());
    leftStrip.setData(leftBuffer);
    leftStrip.start();

    rightStrip.setLength(rightBuffer.getLength());
    rightStrip.setData(rightBuffer);
    rightStrip.start();

    stripStatus = LEDStripStatus.ON;
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
    leftBuffer.setHSV(i, hue, saturation, value);
    rightBuffer.setHSV(i, hue, saturation, value);
  }
  
  public void setRGB(int i, int red, int green, int blue){
    leftBuffer.setRGB(i, red, green, blue);
    rightBuffer.setRGB(i, red, green, blue);
  }

  public int getBufferLength(){
    return leftBuffer.getLength();
  }

  public void sendData(){
    leftStrip.setData(leftBuffer);
    rightStrip.setData(rightBuffer);
  }

  public void stopLEDStrip() {
    leftStrip.stop();
    rightStrip.stop();
    stripStatus = LEDStripStatus.OFF;
  }

  public void startLEDStrip() {
    leftStrip.start();
    rightStrip.start();
    stripStatus = LEDStripStatus.ON;
  }

}
