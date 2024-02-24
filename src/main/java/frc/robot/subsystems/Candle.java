package frc.robot.subsystems;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Candle extends SubsystemBase {
    private final CANdle robot_candle = new CANdle(Constants.Ports.CANDLEID, Constants.Ports.CTRE_CANBUS);
    private Animation toAnimate = null;
    //different types of animations
    public enum AnimationTypes {
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
    public Candle(){
        changeAnimation(AnimationTypes.Empty);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.GRB;
        config.brightnessScalar = 1.0;
        robot_candle.configAllSettings(config);
    }
    public void setPatternDuration(int r, int g, int b) {
        robot_candle.setLEDs(r, g, b);
    }
    public void turnOff(){
        robot_candle.setLEDs(0, 0, 0);
    }

    public void changeAnimation(AnimationTypes toChange) {

        int LEDS_PER_ANIMATION = 30; // placeholder value can be configured
        switch(toChange)
        {
            default:
            case ColorFlow:
                toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_PER_ANIMATION, ColorFlowAnimation.Direction.Forward);
                break;
            case Fire:
                toAnimate = new FireAnimation(0.5, 0.7, LEDS_PER_ANIMATION, 0.8, 0.5);
                break;
            case Larson:
                toAnimate = new LarsonAnimation(0, 255, 46);
                break;
            case Rainbow:
                toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION);
                break;
            case RgbFade:
                toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION);
                break;
            case SingleFade:
                toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_PER_ANIMATION);
                break;
            case Strobe:
                toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, LEDS_PER_ANIMATION);
                break;
            case Twinkle:
                toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_PER_ANIMATION, TwinkleAnimation.TwinklePercent.Percent42);
                break;
            case TwinkleOff:
                toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_PER_ANIMATION, TwinkleOffAnimation.TwinkleOffPercent.Percent76);
                break;
            case Empty:
                toAnimate = null;
                break;
        }
    }

    @Override
    public void periodic(){
        if(toAnimate==null){
            robot_candle.setLEDs(255,0,0); //red alliance
        }
        else{
            robot_candle.animate((toAnimate));
        }
    }

}
