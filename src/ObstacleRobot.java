import simbad.sim.Agent;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Vector3d;

/**
 * 障碍物机器人
 */
public class ObstacleRobot extends Agent {
    RangeSensorBelt bumpers, sonars;
    boolean movable;

    public ObstacleRobot(Vector3d position, String name) {
        super(position, name);
        bumpers = RobotFactory.addBumperBeltSensor(this);
        sonars = RobotFactory.addSonarBeltSensor(this);
    }

    void setMovable(boolean flag) {

        movable = flag;
    }

    public void initBehavior() {

        this.setMovable(true);
    }

    public void performBehavior() {

        if (collisionDetected()) {
            // reads the three front quadrants
            double left = sonars.getFrontLeftQuadrantMeasurement();
            double right = sonars.getFrontRightQuadrantMeasurement();
            double front = sonars.getFrontQuadrantMeasurement();
            if ((front < 1) || (left < 1) || (right < 1)) {
                if (left < right) {
                    setRotationalVelocity(-1 - (0.3 * Math.random()));// 随机向右转
                } else {
                    setRotationalVelocity(1 - (0.3 * Math.random()));// 随机向左转
                }
                setTranslationalVelocity(0);
            }
        } else {
            setTranslationalVelocity(0.5);
            if ((getCounter() % 100) == 0)
                setRotationalVelocity(Math.PI / 2 * (0.5 - Math.random()));
        }
    }

}
