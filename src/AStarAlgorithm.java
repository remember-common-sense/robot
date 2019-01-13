import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AStarAlgorithm {
    private int[][] map;//获取静态地图
    private List<Node> openList;//开启列表
    private List<Node> closeList;//关闭列表
    private final double COST_STRAIGHT = 1;//直方向路径评分
    private final double COST_DIAGONAL = Math.sqrt(2); //斜方向路径评分
    private int row;//行
    private int column;//列
    public List<Node> resultList;
    
    public AStarAlgorithm(int[][] map, int row, int column){
        this.map=map;
        this.row=row;
        this.column=column;
        openList=new ArrayList<Node>();
        closeList=new ArrayList<Node>();
    }
    
    //查找路径 
    public int search(int x1,int y1,int x2,int y2){
        if(x1<0||x1>=column||x2<0||x2>=column||y1<0||y1>=row||y2<0||y2>=row){
            return -1;
        }
        if(map[x1][y1]==0||map[x2][y2]==0){
            return -1;				//地图错误
        }
        Node sNode=new Node(x1,y1,null);    //起始节点
        Node eNode=new Node(x2,y2,null);    //终止节点
        openList.add(sNode);
        resultList=search(sNode, eNode);
        if(resultList.size()==0){
            return 0;        //没有找到路径
        }
        //路径点标记为-1
        for(Node node:resultList){
            map[node.getX()][node.getY()]=-1;
        }
        System.out.println("路径长度:"+resultList.size());
        return 1;					//找到坐标
    }
    
    //节点查找，核心算法
    private List<Node> search(Node sNode,Node eNode){
        List<Node> resultList=new ArrayList<Node>();
        boolean isFind=false;
        Node node=null;
        while(openList.size()>0){
            //取出openlist中F值最低的，用于下一个查找节点
            node=openList.get(0);
            //判断此节点是否为终点
            if(node.getX()==eNode.getX()&&node.getY()==eNode.getY()){
                isFind=true;
                break;
            }
            //查找上一个
            if((node.getY()-1)>=0){
                checkPath(node.getX(),node.getY()-1,node, eNode, COST_STRAIGHT);
            }
            //下
            if((node.getY()+1)<row){
                checkPath(node.getX(),node.getY()+1,node, eNode, COST_STRAIGHT);
            }
            //左边
            if((node.getX()-1)>=0){
                checkPath(node.getX()-1,node.getY(),node, eNode, COST_STRAIGHT);
            }
            //右边
            if((node.getX()+1)<column){
                checkPath(node.getX()+1,node.getY(),node, eNode, COST_STRAIGHT);
            }
            
            
            //右下
            if((node.getX()+1)<column&&(node.getY()+1)<row){
                checkPath(node.getX()+1,node.getY()+1,node, eNode, COST_DIAGONAL);
            }
            //左下
            if((node.getX()-1)>=0&&(node.getY()+1)<row){
                checkPath(node.getX()-1,node.getY()+1,node, eNode, COST_DIAGONAL);
            }
            //右上
            if((node.getX()+1)<column&&(node.getY()-1)>=0){
                checkPath(node.getX()+1,node.getY()-1,node, eNode, COST_DIAGONAL);
            }
            //左上
            if((node.getX()-1)>=0&&(node.getY()-1)>=0){
                checkPath(node.getX()-1,node.getY()-1,node, eNode, COST_DIAGONAL);
            }
//            
            //从openlist中删除，加入到closelist中
            
            closeList.add(openList.remove(0));
            //F值排序
            Collections.sort(openList, new NodeFComparator());
        }
        if(isFind){
            getPath(resultList, node);
        }
        return resultList;
    }
    
    //查询此路是否能走通
    private boolean checkPath(int x,int y,Node parentNode,Node eNode,double cost){
        Node node=new Node(x, y, parentNode);
        //判断地图是否能通过
        if(map[x][y]==0||map[x][y]==3){
            closeList.add(node);
            return false;
        }
        //查看是否在closelist列表中
        if(isListContains(closeList, x, y)!=-1){
            return false;
        }
        //查看是否在在openlist中
        int index=-1;
        if((index=isListContains(openList, x, y))!=-1){
            //更新G值ֵ
            if((parentNode.getG()+cost)<openList.get(index).getG()){
                node.setParentNode(parentNode);
                countG(node, eNode, cost);
                countF(node);
                openList.set(index, node);
            }
        }else{
            //添加到openlist
            node.setParentNode(parentNode);
            count(node, eNode, cost);
            openList.add(node);
        }
        return true;
    }
    
    //判断列表中是否包含某个节点
    private int isListContains(List<Node> list,int x,int y){
        for(int i=0;i<list.size();i++){
            Node node=list.get(i);
            if(node.getX()==x&&node.getY()==y){
                return i;
            }
        }
        return -1;
    }
    
    //递归得到路径
    private void getPath(List<Node> resultList,Node node){
        if(node.getParentNode()!= null){
            getPath(resultList, node.getParentNode());
        }
        resultList.add(node);
    }
    
    //计算G,H,Fֵ
    private void count(Node node,Node eNode,double cost){
        countG(node, eNode, cost);
        countH(node, eNode);
        countF(eNode);
    }
    //Gֵ
    private void countG(Node node,Node eNode,double cost){
        if(node.getParentNode()==null){
            node.setG(cost);
        }else{
            node.setG(node.getParentNode().getG()+cost);
        }
    }
    //Hֵ
    private void countH(Node node,Node eNode){
        node.setH(Math.abs(node.getX()-eNode.getX())+Math.abs(node.getY()-eNode.getY()));
    }
    //Fֵ
    private void countF(Node node){
        node.setF(node.getG()+node.getF());
    }
    

}

