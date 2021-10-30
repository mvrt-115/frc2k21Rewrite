// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  public AddressableLED led;
  public AddressableLEDBuffer ledBuffer;

  public LED(int len) 
  {
    led = new AddressableLED(Constants.LED.LED_PORT);
    ledBuffer = new AddressableLEDBuffer(len);
    led.setLength(ledBuffer.getLength());

    led.start();
  }

  //Starts at startIndex and ends (not including) at endIndex
  public void setOneColor(Color color, int startIndex, int endIndex)
  {
    for(int i = startIndex; i < ledBuffer.getLength(); i++)
    {
      ledBuffer.setLED(i, color);
    }

    led.setData(ledBuffer);
  }

  public void setTwoColors(Color color1, Color color2, int colorBlockLength, int startIndex, int endIndex)
  {
    int count = 0;
    Color currColor = color1;
    for(int i = startIndex; i < endIndex; i+=colorBlockLength)
    {
      if(count%colorBlockLength == 0)
      {
        if(currColor.equals(color1))
          currColor = color2;
        else if(currColor.equals(color2))
          currColor = color1;
      }

      setOneColor(currColor, i, i+colorBlockLength);
    }

    led.setData(ledBuffer);
  }

  public void setRainbow()
  {
    int counter = 0, redGradient = 255, greenGradient = 0, blueGradient = 0;
    int colorChange = ledBuffer.getLength()/3;
    int gradientChange = 255/colorChange;

    for(int i = 0; i < ledBuffer.getLength(); i++)
    {
      if(counter <= colorChange)
      {
        redGradient -= gradientChange;
        blueGradient += gradientChange;
      }
      else if(counter <= 2 * colorChange)
      {
        blueGradient -= gradientChange;
        greenGradient += gradientChange;
      }
      else if(counter < ledBuffer.getLength())
      {
        greenGradient -= gradientChange;
        redGradient += gradientChange;
      }

      ledBuffer.setRGB(counter, redGradient, greenGradient, blueGradient);
      counter++;
    }

    led.setData(ledBuffer);
  }

  public void moveLEDs(double moveCount)
  {
    Color tempColor = ledBuffer.getLED(0);
    ledBuffer.setLED(0, ledBuffer.getLED(ledBuffer.getLength()-1));
    for(int i = 1; i < ledBuffer.getLength()-1; i++)
    {
      Color currColor = ledBuffer.getLED(i);
      ledBuffer.setLED(i, tempColor);
      tempColor = currColor;
    }

    led.setData(ledBuffer);
    Timer.delay(moveCount);
  }

  public void stopLEDs()
  {
    for(int i = 0; i < ledBuffer.getLength(); i++)
    {
      ledBuffer.setLED(i, Color.kBlack);
    }

    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public int getLength()
  {
    return ledBuffer.getLength();
  }
}




/*
Ideas for LED:
  Basic LED: fills some amount of the leds based on how many balls are in hopper using MVRT,
    if hopper is full, use MVRT move

  Have LEDs move green when AutoAlign aligns the bot within a certain error, 
    otherwise have move yellow while AutoAligning
    If time, make it gradient so that as it turns green from yellow as it enters aligned position

  Have different LED pattern for hanging (make rainbow for hanging, moving MVRT for full hopper)
*/
