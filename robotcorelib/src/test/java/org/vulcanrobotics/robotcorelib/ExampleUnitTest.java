package org.vulcanrobotics.robotcorelib;

import org.junit.Test;
import org.vulcanrobotics.robotcorelib.math.Functions;
import org.vulcanrobotics.robotcorelib.math.Point;

import java.util.ArrayList;

import static org.junit.Assert.*;

/**
 * Example local unit test, which will execute on the development machine (host).
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */
public class ExampleUnitTest {
    @Test
    public void addition_isCorrect() {
        assertEquals(4, 2 + 2);
    }

    @Test
    public void lineCircleIntersectTest() {
        ArrayList<Point> testResult = new ArrayList<>();

        testResult.add(new Point(0.707, 0.707));
        testResult.add(new Point(-0.707, -0.707));

        assertEquals(Functions.lineCircleIntersect(new Point(-1, -1), new Point(1, 1), 1, new Point(0, 0)).get(0).x, testResult.get(0).x, 0.1);
    }

}