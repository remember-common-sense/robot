import simbad.sim.Box;
import simbad.sim.EnvironmentDescription;
import simbad.sim.Wall;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.util.ArrayList;
import java.util.List;

public class MyEnv extends EnvironmentDescription {

    //地图
    public static int[][] map = {
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},     //1
            {1,3,1,1,3,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1},
            {1,1,3,3,1,3,1,1,1,3,1,1,1,3,0,0,0,3,1,1},
            {1,1,1,1,1,1,1,1,1,1,1,1,1,3,0,0,0,3,1,1},		//5
            {1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1},
            {1,1,3,3,1,3,0,0,0,3,1,1,1,1,1,1,0,0,1,1},
            {1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,3,0,0,3,1},
            {1,1,0,0,1,1,1,1,1,3,1,1,3,1,1,1,0,0,1,1},
            {1,1,3,3,1,1,1,1,1,1,1,1,1,1,1,1,3,3,1,1},		//10
            {1,1,1,1,1,1,0,0,1,1,3,3,1,1,1,1,1,1,1,1},
            {1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,3,1,3,1,1},
            {1,1,1,0,1,1,1,1,3,1,0,0,3,1,1,0,0,0,1,1},
            {1,3,1,1,3,1,1,1,3,1,0,0,3,1,1,0,0,0,1,1},
            {1,1,1,1,1,1,3,3,1,1,1,1,1,1,3,1,1,1,3,1},		//15
            {1,3,1,1,3,1,0,0,1,1,3,3,1,1,1,1,1,1,1,1},
            {1,1,1,1,1,1,0,0,1,1,0,0,1,1,1,3,1,1,1,1},
            {1,1,1,0,0,1,3,3,1,1,0,0,1,1,0,1,1,1,1,1},
            {1,1,1,1,1,1,1,1,1,1,3,3,1,1,1,1,1,1,1,1},
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,1,1,1,1}};		//20

    //起点坐标
    public static final Vector3d start3d = new Vector3d(-9, 0, -9);

    //终点坐标
    public static final Vector2d end2d = new Vector2d(9, 9);
    public static final Vector3d end3d = new Vector3d(9, 0, 9);

    //引力系数、斥力系数
    public static final double repelConstant = 1.0;
    public static final double attractConstant = 30.0;

    /**
     * 地图、机器人初始化
     */
    public MyEnv() {
        //地图边界
        Wall w1 = new Wall(new Vector3d(10, 0, 0), 20, 1, this);
        w1.rotate90(1);
        add(w1);
        Wall w2 = new Wall(new Vector3d(-10, 0, 0), 20, 1, this);
        w2.rotate90(1);
        add(w2);
        Wall w3 = new Wall(new Vector3d(0, 0, 10), 20, 1, this);
        add(w3);
        Wall w4 = new Wall(new Vector3d(0, 0, -10), 20, 1, this);
        add(w4);

        int row = 20;
        int column = 20;
        //地图箱型障碍物障碍物
        for(int i=0;i<row;i++)
            for(int j=0;j<column;j++){
                if(map[i][j]==0){
                    Box b = new Box(new Vector3d(j-column/2, 0, i-row/2), new Vector3f(1, 1, 1),
                            this);
                    add(b);
                }
            }

        //障碍物机器人
        add(new ObstacleRobot(new Vector3d(6, 0, 7), "ObstacleRobot 1"));
        add(new ObstacleRobot(new Vector3d(-6, 0, -5), "ObstacleRobot 2"));
        add(new ObstacleRobot(new Vector3d(2, 0, -2), "ObstacleRobot 3"));


        //将动态障碍物暂时视为静态障碍物
        map[5][4] = 0;
        map[14][15] = 0;

        //执行A*算法
        AStarAlgorithm aStar = new AStarAlgorithm(map, row, column);
        aStar.search((int)start3d.x + 10, (int)start3d.z + 10, (int)end2d.x + 10, (int)end2d.y + 10);
        List<Node> resultList = aStar.resultList;

        //采用人工势场法的机器人
        add(new APFRobot(start3d, "APF Robot"));

        //采用A*算法的机器人
        add(new AStarRobot(start3d, "AStart Robot", column, row, map, (int)end2d.x, (int)end2d.y));

        //设置四个中间节点
        List<Node> list = new ArrayList<>();
        int n = resultList.size() / 4;
        for(int i = 1; i <= 4; i++) {
            list.add(resultList.get(i * n));
        }
        list.add(resultList.get(resultList.size() - 1));

        //结合A*算法和人工势场法的机器人
        add(new MyRobot(start3d, "My Robot", list));
    }

}
