package org.firstinspires.ftc.teamcode.display;

import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Color;
import team.techtigers.core.display.sprites.numbers.OneSprite;
import team.techtigers.core.display.sprites.numbers.TwoSprite;
import team.techtigers.core.display.sprites.numbers.ThreeSprite;
import team.techtigers.core.display.sprites.numbers.FourSprite;
import team.techtigers.core.display.sprites.numbers.FiveSprite;
import team.techtigers.core.display.sprites.numbers.SixSprite;
import team.techtigers.core.display.sprites.numbers.SevenSprite;
import team.techtigers.core.display.sprites.numbers.EightSprite;
import team.techtigers.core.display.sprites.numbers.NineSprite;
import team.techtigers.core.display.sprites.numbers.ZeroSprite;
import team.techtigers.core.display.sprites.Sprite;

public class BlockRegion extends DisplayRegion {
    private final OneSprite numOne;
    private final TwoSprite numTwo;
    private final ThreeSprite numThree;
    private final FourSprite numFour;
    private final FiveSprite numFive;
    private final SixSprite numSix;
    private final SevenSprite numSeven;
    private final EightSprite numEight;
    private final NineSprite numNine;
    private final ZeroSprite numZero;
    private final OneSprite numOne2;
    private final TwoSprite numTwo2;
    private final ThreeSprite numThree2;
    private final FourSprite numFour2;
    private final FiveSprite numFive2;
    private final SixSprite numSix2;
    private final SevenSprite numSeven2;
    private final EightSprite numEight2;
    private final NineSprite numNine2;
    private final ZeroSprite numZero2;
    private final Sprite[] sprites;
    private long startTime = 0;
    private int phase = 0; // 0 = 30s countdown, 1 = 8s wait, 2 = 90s countdown
    private long phaseStartTime = 0;

    public BlockRegion(int x, int y) {
        super(x, y, 8, 8);
        numOne = new OneSprite(0, 1);
        numOne2 = new OneSprite(5, 1);
        numTwo = new TwoSprite(0, 1);
        numTwo2 = new TwoSprite(5, 1);
        numThree = new ThreeSprite(0, 1);
        numThree2 = new ThreeSprite(5, 1);
        numFour = new FourSprite(0, 1);
        numFour2 = new FourSprite(5, 1);
        numFive = new FiveSprite(0, 1);
        numFive2 = new FiveSprite(5, 1);
        numSix = new SixSprite(0, 1);
        numSix2 = new SixSprite(5, 1);
        numSeven = new SevenSprite(0, 1);
        numSeven2 = new SevenSprite(5, 1);
        numEight = new EightSprite(0, 1);
        numEight2 = new EightSprite(5, 1);
        numNine = new NineSprite(0, 1);
        numNine2 = new NineSprite(5, 1);
        numZero = new ZeroSprite(0, 1);
        numZero2 = new ZeroSprite(5, 1);

        numOne.setColor(Color.BLUE);
        numOne2.setColor(Color.BLUE);
        numTwo.setColor(Color.BLUE);
        numTwo2.setColor(Color.BLUE);
        numThree.setColor(Color.BLUE);
        numThree2.setColor(Color.RED);
        numFour.setColor(Color.BLUE);
        numFour2.setColor(Color.BLUE);
        numFive.setColor(Color.BLUE);
        numFive2.setColor(Color.BLUE);
        numSix.setColor(Color.BLUE);
        numSix2.setColor(Color.BLUE);
        numSeven.setColor(Color.BLUE);
        numSeven2.setColor(Color.BLUE);
        numEight.setColor(Color.BLUE);
        numEight2.setColor(Color.BLUE);
        numNine.setColor(Color.BLUE);
        numNine2.setColor(Color.BLUE);
        numZero.setColor(Color.BLUE);
        numZero2.setColor(Color.BLUE);


        sprites = new Sprite[]{numThree, numZero2};

    }
    private Sprite getDigitSprite(int digit, boolean second) {
        switch (digit) {
            case 0: return second ? numZero2 : numZero;
            case 1: return second ? numOne2 : numOne;
            case 2: return second ? numTwo2 : numTwo;
            case 3: return second ? numThree2 : numThree;
            case 4: return second ? numFour2 : numFour;
            case 5: return second ? numFive2 : numFive;
            case 6: return second ? numSix2 : numSix;
            case 7: return second ? numSeven2 : numSeven;
            case 8: return second ? numEight2 : numEight;
            case 9: return second ? numNine2 : numNine;
            default: return second ? numZero2 : numZero;
        }
    }

    private void setAllColors(Color color) {
        numOne.setColor(color);   numOne2.setColor(color);
        numTwo.setColor(color);   numTwo2.setColor(color);
        numThree.setColor(color); numThree2.setColor(color);
        numFour.setColor(color);  numFour2.setColor(color);
        numFive.setColor(color);  numFive2.setColor(color);
        numSix.setColor(color);   numSix2.setColor(color);
        numSeven.setColor(color); numSeven2.setColor(color);
        numEight.setColor(color); numEight2.setColor(color);
        numNine.setColor(color);  numNine2.setColor(color);
        numZero.setColor(color);  numZero2.setColor(color);
    }

    @Override
    public void update() {
        long now = System.currentTimeMillis();

        if (startTime == 0) {
            startTime = now;
            phaseStartTime = now;
        }

        long phaseElapsed = now - phaseStartTime;

        if (phase == 0) {
            long remaining = Math.max(0, 31_000 - phaseElapsed);
            int seconds = (int) (remaining / 1000);
            sprites[0] = getDigitSprite(seconds / 10, false);
            sprites[1] = getDigitSprite(seconds % 10, true);
            setAllColors(Color.BLUE);
            if (remaining == 0) {
                phase = 1;
                phaseStartTime = now;
            }
        } else if (phase == 1) {
            sprites[0] = getDigitSprite(0, false);
            sprites[1] = getDigitSprite(0, true);
            setAllColors(Color.BLACK);
            if (phaseElapsed >= 8_000) {
                phase = 2;
                phaseStartTime = now;
                setAllColors(Color.BLUE);
            }
        } else {
            long remaining = Math.max(0, 91_000 - phaseElapsed);
            int seconds = (int) (remaining / 1000);
            sprites[0] = getDigitSprite(seconds / 10, false);
            sprites[1] = getDigitSprite(seconds % 10, true);
            if (remaining == 0) {
                setAllColors(Color.RED);
            } else {
                setAllColors(Color.BLUE);
            }
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }



}
