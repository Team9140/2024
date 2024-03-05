package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Candle extends SubsystemBase {
  private final CANdle candle = new CANdle(Constants.Ports.CANDLEID, Constants.Ports.CTRE_CANBUS);

  private Animation toAnimate;
  private double animationDuration;
  private boolean isLedAnimated; // variable that checks if a LED action that is not an animation is occurring

  // Different types of animations
  public static enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    Empty
  }

  public Candle() {
    this.changeAnimation(AnimationTypes.Empty, 0);

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = CANdle.LEDStripType.GRB;
    config.brightnessScalar = Constants.CANDLE_BRIGHTNESS;
    this.candle.configAllSettings(config);
  }

  public void setColor(int r, int g, int b) {
    changeAnimation(AnimationTypes.Empty, 0);
    isLedAnimated = true;
    candle.setLEDs(r, g, b);
  }

  public void setColor(int r, int g, int b, double duration) {
    changeAnimation(AnimationTypes.Empty, 0);
    isLedAnimated = true;
    candle.setLEDs(r, g, b);
    // Create a WaitCommand with the specified duration
    WaitCommand waitCommand = new WaitCommand(duration);
    CommandScheduler.getInstance().schedule(waitCommand);
    waitCommand.andThen(() -> changeAnimation(AnimationTypes.Empty, 0));
    isLedAnimated = false;
  }

  public void turnOff(){
    isLedAnimated = true;
    candle.setLEDs(0, 0, 0);
  }

  public void changeAnimation(AnimationTypes animation, double duration) {
    isLedAnimated = false;
    animationDuration = duration;
    switch (animation) {
      default:
      case Empty:
        this.toAnimate = null;
        break;
      case ColorFlow:
        this.toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, Constants.CANDLE_LEDS_PER_ANIMATION, ColorFlowAnimation.Direction.Forward);
        break;
      case Fire:
        this.toAnimate = new FireAnimation(0.5, 0.7, Constants.CANDLE_LEDS_PER_ANIMATION, 0.8, 0.5);
        break;
      case Larson:  // Mr. Larson reference???
        this.toAnimate = new LarsonAnimation(0, 255, 46);
        break;
      case Rainbow:
        this.toAnimate = new RainbowAnimation(1, 0.7, Constants.CANDLE_LEDS_PER_ANIMATION);
        break;
      case RgbFade:
        this.toAnimate = new RgbFadeAnimation(0.7, 0.4, Constants.CANDLE_LEDS_PER_ANIMATION);
        break;
      case SingleFade:
        this.toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANDLE_LEDS_PER_ANIMATION);
        break;
      case Strobe:
        this.toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, Constants.CANDLE_LEDS_PER_ANIMATION);
        break;
      case Twinkle:
        this.toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, Constants.CANDLE_LEDS_PER_ANIMATION, TwinkleAnimation.TwinklePercent.Percent42);
        break;
      case TwinkleOff:
        this.toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, Constants.CANDLE_LEDS_PER_ANIMATION, TwinkleOffAnimation.TwinkleOffPercent.Percent76);
        break;
    }
  }

  @Override
  public void periodic() {
    if (this.toAnimate != null) {
      if(animationDuration==0) {
        // Play set animation
        this.candle.animate(this.toAnimate);
      }
      else{
        this.candle.animate(this.toAnimate);
        WaitCommand waitCommand = new WaitCommand(animationDuration);
        CommandScheduler.getInstance().schedule(waitCommand);
        waitCommand.andThen(() -> changeAnimation(AnimationTypes.Empty, 0));
      }
    } else if (isLedAnimated) {
      // do nothing the LED color is set to a color or turned off
    } else if (Constants.alliance.isPresent() && Constants.alliance.get().equals(DriverStation.Alliance.Red)) {
      // Red alliance color
      this.candle.setLEDs(255,0,0);
    } else {
      // Blue alliance color
      this.candle.setLEDs(0,0,255);
    }
  }
}
