package org.team100.lib.field;

import org.junit.jupiter.api.Test;
import org.team100.lib.field.FieldConstants.ReefPoint;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Check the positions of the scoring points.
 * 
 * See chart:
 * 
 * https://docs.google.com/spreadsheets/d/1I3ZZU4iZdwebHROykgXosJXGss2YPVTT3JEtHqNjdpQ
 */
public class SetpointsTest {
    private static final boolean DEBUG = false;

    void print(Translation2d t) {
        if (DEBUG)
            System.out.printf("%6.3f %6.3f\n", t.getX(), t.getY());
    }

    @Test
    void testSetA() {
        if (DEBUG)
            System.out.println("x, y");
        print(FieldConstants.getScoringDestination(ReefPoint.A, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.AB, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.B, 1.4));

        print(FieldConstants.getScoringDestination(ReefPoint.C, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.CD, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.D, 1.4));

        print(FieldConstants.getScoringDestination(ReefPoint.E, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.EF, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.F, 1.4));

        print(FieldConstants.getScoringDestination(ReefPoint.G, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.GH, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.H, 1.4));

        print(FieldConstants.getScoringDestination(ReefPoint.I, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.IJ, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.J, 1.4));

        print(FieldConstants.getScoringDestination(ReefPoint.K, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.KL, 1.4));
        print(FieldConstants.getScoringDestination(ReefPoint.L, 1.4));
    }
}
