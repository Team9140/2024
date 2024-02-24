import frc.robot.subsystems.SwerveModule;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SwerveModuleTest {
    static final double DELTA = 1e-2;

    @Test
    void calculatesBestTurn() {
        assertEquals(2 * Math.PI, SwerveModule.bestTurn(0, 3 * Math.PI / 2), DELTA);
        assertEquals(0, SwerveModule.bestTurn(0, 0), DELTA);
        assertEquals(-3 * Math.PI / 2, SwerveModule.bestTurn(Math.PI / 2, -3 * Math.PI / 2), DELTA);
        assertEquals(Math.PI/2, SwerveModule.bestTurn(Math.PI/2, 0), DELTA);
    }
}
