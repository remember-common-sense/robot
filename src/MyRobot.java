import simbad.sim.Agent;
import simbad.sim.LampActuator;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import java.util.List;

/**
 * 结合人工势场法和A*算法的机器人
 */
public class MyRobot extends Agent {

    //机器人传感器
    RangeSensorBelt sonars;
    RangeSensorBelt bumpers;
    LampActuator lamp;

    //起点位置
    Vector3d start3d;

    //开始时间
    long startTime;

    //当前时间
    long currentTime;

    //结束时间
    long endTime;

    //到达终点标记
    boolean flag;
    double curForce;

    //当前目标点
    List<Node> list;
    Node curNode;
    int index;

    int freeze_time;

    public MyRobot(Vector3d position, String name, List<Node> list) {
        super(position, name);
        bumpers = RobotFactory.addBumperBeltSensor(this);
        sonars = RobotFactory.addSonarBeltSensor(this);
        lamp = RobotFactory.addLamp(this);
        start3d = position;
        flag = false;
        curForce = -1;
        setColor(new Color3f(8, 0, 8));
        this.list = list;
        this.curNode = list.get(0);
        index = 0;
        this.setColor(new Color3f(4, 0, 4));
        freeze_time = 0;
    }

    /**
     * 获取机器人速度
     * @return
     */
    public Vector3d getVeclocity() {
        return this.linearVelocity;
    }

    /**
     * 获取机器人所处位置象限
     * @param vector
     * @return
     */
    private int getQuadrant(Vector2d vector) {
        double x = vector.x;
        double y = vector.y;
        if(x > 0 && y > 0) {
            //第一象限
            return 1;
        } else if(x < 0 && y > 0) {
            //第二象限
            return 2;
        } else if(x < 0 && y < 0) {
            //第三象限
            return 3;
        } else if(x > 0 && y < 0) {
            //第四象限
            return 4;
        } else if(x > 0 && y == 0) {
            //x轴正半轴
            return -1;
        } else if(x == 0 && y > 0) {
            //y轴正半轴
            return -2;
        } else if(x < 0 && y == 0) {
            //x轴负半轴
            return -2;
        } else if(x == 0 && y < 0) {
            //y轴负半轴
            return -2;
        } else {
            return 0;
        }
    }

    /**
     * 获取角度向量
     * @param v1
     * @param v2
     * @return
     */
    private double getAngle(Vector2d v1, Vector2d v2) {
        double k = v1.y / v1.x;
        double y = k * v2.x;
        switch (getQuadrant(v1))
        {
            case 1:
            case 4:
            case -1:
                if (v2.y > y)
                {
                    return v1.angle(v2);
                } else if (v2.y < y)
                {
                    return 2 * Math.PI - v1.angle(v2);
                } else
                {
                    if (v1.x * v2.x < 0)
                    {
                        return Math.PI;
                    } else
                    {
                        return 0;
                    }
                }
            case 2:
            case 3:
            case -3:
                if (v2.y > y) {
                    return 2 * Math.PI - v1.angle(v2);
                } else if (v2.y < y) {
                    return v1.angle(v2);
                } else {
                    if (v1.x * v2.x < 0) {
                        return Math.PI;
                    } else {
                        return 0;
                    }
                }
            case -2:
                int i = getQuadrant(v2);
                if (i == -4)
                {
                    return Math.PI;
                } else if (i == -2 || i == -1 || i == 1 || i == 4)
                {
                    return 2 * Math.PI - v1.angle(v2);
                } else
                {
                    return v1.angle(v2);
                }
            case -4:
                int j = getQuadrant(v2);
                if (j == -1)
                {
                    return Math.PI;
                } else if (j == -4 || j == -1 || j == 1 || j == 4)
                {
                    return v1.angle(v2);
                } else
                {
                    return 2 * Math.PI - v1.angle(v2);
                }
            default:
                return -1;
        }
    }

    private Vector2d transform(Vector2d v, Vector2d point)
    {
        Vector2d global = new Vector2d(1, 0);
        double alfa = getAngle(global, v);
        double beta = getAngle(point, v);

        double k1 = Math.cos(alfa + beta) / Math.cos(beta);
        double k2 = Math.sin(alfa + beta) / Math.sin(beta);

        double x = point.x * k1;
        double y = point.y * k2;

        return new Vector2d(x, y);

    }

    /**
     * 计算斥力
     * @param distance
     * @param range
     * @return
     */
    private double repelForce(double distance, double range)
    {
        double force = 0;
        Point3d p = new Point3d();
        getCoords(p);
        Vector2d pos = new Vector2d(p.z, p.x);
        Vector2d toGoal = new Vector2d((MyEnv.end2d.x - pos.x),
                (MyEnv.end2d.y - pos.y));
        double disGoal = toGoal.length();
        double n=0.5;
        if (distance <= range)
        {
            force = - MyEnv.repelConstant / (distance * distance);
        }

        return force;
    }

    /**
     * 计算引力
     * @param distance
     * @return
     */
    private double attractForce(double distance) {
        double force = MyEnv.attractConstant / distance /distance;
        return force;
    }

    /**
     * 判断是否到达终点
     * @return
     */
    private boolean checkGoal() {
        // get the current postion of the robot
        Point3d currentPos = new Point3d();
        getCoords(currentPos);
        Point3d goalPos = new Point3d(MyEnv.end3d.x, MyEnv.end3d.y, MyEnv.end3d.z);

        if (currentPos.distance(goalPos) <= 0.9) // 如果当前距离目标点小于0.9那么即认为是到达
        {
            return true;
        } else
        {
            return false;
        }
    }

    /**
     * 判断是否陷入无法移动
     * @param pre
     * @param cur
     * @return
     */
    private boolean isStopForLong(Vector2d pre, Vector2d cur) {
        if (cur.equals(pre)) {
            if (freeze_time == 20)
                // 如果边界设置太小会影响正常的碰撞处理
                return true;
            else {
                freeze_time++;
                return false;
            }
        } else {
            freeze_time = 0;
            return false;
        }
    }

    /**
     *
     * @param curNode
     * @return
     */
    private boolean checkNode(Node curNode) {
        Point3d currentPos = new Point3d();
        getCoords(currentPos);
        Point3d nodePos = new Point3d(curNode.getY()-10, 0, curNode.getX()-10);

        if (currentPos.distance(nodePos) <= 0.5 && index < list.size() - 1) {
            return true;
        } else {
            return false;
        }
    }

    public void initBehavior() {
        startTime = System.currentTimeMillis();
    }

    public void performBehavior() {
        // 为了防止智能体剧烈晃动，每20帧计算一次受力
        if (getCounter() % 20 == 0)
        {
            //获取机器人移动速度
            Vector3d velocity = getVeclocity();

            //获取机器人移动方向
            Vector2d direct = new Vector2d(velocity.z, velocity.x);

            //获取当前运动位置
            Point3d p = new Point3d();
            getCoords(p);
            Vector2d pos = new Vector2d(p.z, p.x);

            // 获取前一次运动位置
            Point3d p1 = new Point3d();
            getCoords(p1);
            Vector2d pre_pos = new Vector2d(p.z, p.x);

            //获取三个方位障碍物的距离
            double d0 = sonars.getMeasurement(0) + (double)getRadius();// front声纳，正前方
            double d1 = sonars.getMeasurement(1) + (double)getRadius();// frontleft声纳，左前方
            double d2 = sonars.getMeasurement(8) + (double)getRadius();// frontright声纳，右前方

            //计算三个方向障碍物的斥力
            double rf0 = repelForce(d0, 1.0);
            double rf1 = repelForce(d1, 1.0);
            double rf2 = repelForce(d2, 1.0);

            //计算三个方向障碍物斥力的合力
            double k1 = Math.cos(2 * Math.PI / 9);
            double k2 = Math.sin(2 * Math.PI / 9);
            Vector2d vf0 = new Vector2d(0 - rf0, 0);
            Vector2d vf1 = new Vector2d((0 - rf1 * k1), (0 - rf1 * k2));
            Vector2d vf2 = new Vector2d((rf2 * k1), (rf2 * k2));
            Vector2d composition = new Vector2d();
            composition.setX(vf0.x + vf1.x + vf2.x);
            composition.setY(vf0.y + vf1.y + vf2.y);

            //计算合力斥力向量
            Vector2d repelForceVector = transform(direct, composition);

            Vector2d toGoal = new Vector2d((curNode.getX() - 10 - pos.x),
                    (curNode.getY() - 10 - pos.y));
            double disGoal = toGoal.length();

            double goalForce = attractForce(disGoal);

            Vector2d goalForceVector = new Vector2d(
                    (goalForce * toGoal.x / disGoal),
                    (goalForce * toGoal.y / disGoal));
            //Vector2d originForceVector = new Vector2d(start3d.x, start3d.z);

            double x = repelForceVector.x + goalForceVector.x;
            double y = repelForceVector.y + goalForceVector.y;

            //斥力和引力的合力
            Vector2d allForces = new Vector2d(x, y);

            double angle = getAngle(direct, allForces);

            // System.out.println(0.0005*Math.random());
            if(allForces.length() <= 10) {
                angle = angle * (1 - 0.2 * Math.random());
            }

            curForce = allForces.length();

            // 判断转动方向
            if (angle < Math.PI)
            {
                setRotationalVelocity(angle);
            } else if (angle > Math.PI)
            {
                setRotationalVelocity((angle - 2 * Math.PI));
            }

            if(checkNode(curNode)) {
                this.index++;
                this.curNode = list.get(index);
//                System.out.println(curNode.getX() + " " + curNode.getY());
                return;
            }

            if (checkGoal())
            {
                // 到达目标点，停止运动
                setTranslationalVelocity(0);
                setRotationalVelocity(0);
                lamp.setOn(true);
                endTime = System.currentTimeMillis();
                if(!flag)
                    System.out.println("结合人工势场法和A*算法机器人" + "耗费时间" + (endTime - startTime) + "ms" + "; " + "运动里程：" + getOdometer() + "m;");
                flag = true;
                return;
            } else
            {
                lamp.setOn(false);
                setTranslationalVelocity(0.5);
            }

            // 检测是否碰撞
            if (bumpers.oneHasHit())
            {
                lamp.setBlink(true);
                // reads the three front quadrants
                double left = sonars.getFrontLeftQuadrantMeasurement();
                double right = sonars.getFrontRightQuadrantMeasurement();
                double front = sonars.getFrontQuadrantMeasurement();
                // if obstacle near
                if ((front < 1) || (left < 1) || (right < 1))
                {
                    if (left < right)
                    {
                        setRotationalVelocity(-1 - (0.1 * Math.random()));// 随机向右转
                    } else
                    {
                        setRotationalVelocity(1 - (0.1 * Math.random()));// 随机向左转
                    }
                    setTranslationalVelocity(0);
                }
            }
            else
                lamp.setBlink(false);

            // 判断是否一直不动
            // 获取前一次运动位置
            getCoords(p);
            Vector2d cur_pos = new Vector2d(p.z, p.x);
            if (isStopForLong(pre_pos, cur_pos)) {
                setTranslationalVelocity(0);
                setRotationalVelocity(Math.PI );
                freeze_time = 0;
            }
        }
    }
}
