package org.team100.lib.optimization;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.profile.timed.JerkLimitedTimedProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseInterpolationTest {
    private static final boolean DEBUG = false;

    @Test
    void test1() {
        // does WPI pose interpolation make straight lines? no.
        Pose3d start = new Pose3d();
        Pose3d end = new Pose3d(1, 1, 0, new Rotation3d(0, 0, -3));
        double distance = start.getTranslation().getDistance(end.getTranslation());
        JerkLimitedTimedProfile profile = new JerkLimitedTimedProfile(1, 5, 20, true);
        profile.init(new Control100(), new Model100(distance, 0));
        int i = 0;
        for (double t = 0; t < 2; t += 0.02) {
            Control100 c = profile.sample(t);
            double s = c.x() / distance;
            Pose3d setpoint = start.interpolate(end, s);
            if (DEBUG)
                System.out.printf("%d, %.8e, %.8e, %s\n", i, t, s, StrUtil.poseStr(setpoint));
            ++i;
        }
    }

    @Test
    void test1b() {
        // does my pose interpolation make straight lines? yes. :-)
        Pose3d start = new Pose3d();
        Pose3d end = new Pose3d(1, 1, 0, new Rotation3d(0, 0, -3));
        double distance = start.getTranslation().getDistance(end.getTranslation());
        JerkLimitedTimedProfile profile = new JerkLimitedTimedProfile(1, 5, 20, true);
        profile.init(new Control100(), new Model100(distance, 0));
        int i = 0;
        for (double t = 0; t < 2; t += 0.02) {
            Control100 c = profile.sample(t);
            double s = c.x() / distance;
            Pose3d setpoint = GeometryUtil.interpolate(start, end, s);
            if (DEBUG)
                System.out.printf("%d, %.8e, %.8e, %s\n", i, t, s, StrUtil.poseStr(setpoint));
            ++i;
        }
    }

    @Test
    void test2() {
        // does WPI rotation interpolation make straight lines? no.
        Rotation3d start = new Rotation3d();
        Rotation3d end = new Rotation3d(0, 1, -3);
        double distance = 1.5;
        JerkLimitedTimedProfile profile = new JerkLimitedTimedProfile(1, 5, 20, true);
        profile.init(new Control100(), new Model100(distance, 0));
        int i = 0;
        for (double t = 0; t < 2; t += 0.02) {
            Control100 c = profile.sample(t);
            double s = c.x() / distance;
            Rotation3d setpoint = start.interpolate(end, s);
            if (DEBUG)
                System.out.printf("%d, %.8e, %.8e, %s\n", i, t, s, StrUtil.rotStr(setpoint));
            ++i;
        }
    }

    @Test
    void test2b() {
        // does my rotation interpolation make straight lines? yes. :-)
        Rotation3d start = new Rotation3d();
        Rotation3d end = new Rotation3d(0, 1, -3);
        double distance = 1.5;
        JerkLimitedTimedProfile profile = new JerkLimitedTimedProfile(1, 5, 20, true);
        profile.init(new Control100(), new Model100(distance, 0));
        int i = 0;
        for (double t = 0; t < 2; t += 0.02) {
            Control100 c = profile.sample(t);
            double s = c.x() / distance;
            Rotation3d setpoint = GeometryUtil.interpolate(start, end, s);
            if (DEBUG)
                System.out.printf("%d, %.8e, %.8e, %s\n", i, t, s, StrUtil.rotStr(setpoint));
            ++i;
        }
    }

    @Test
    void test3() {
        // does translation interpolation make straight lines? yes.
        Translation3d start = new Translation3d();
        Translation3d end = new Translation3d(0, 1, -3);
        double distance = 1.5;
        JerkLimitedTimedProfile profile = new JerkLimitedTimedProfile(1, 5, 20, true);
        profile.init(new Control100(), new Model100(distance, 0));
        int i = 0;
        for (double t = 0; t < 2; t += 0.02) {
            Control100 c = profile.sample(t);
            double s = c.x() / distance;
            Translation3d setpoint = start.interpolate(end, s);
            if (DEBUG)
                System.out.printf("%d, %.8e, %.8e, %s\n", i, t, s, StrUtil.transStr(setpoint));
            ++i;
        }
    }
}
