/**
 * A*算法中的节点
 */
public class Node {
	private int x;//X坐标
	  private int y;//Y坐标
	  private Node parentNode;//父类节点
	  private double g;//当前点到起点的移动耗费
	  private double h;//当前点到终点的移动耗费，即曼哈顿距离|x1-x2|+|y1-y2|(忽略障碍物)
	  private double f;//f=g+h
  
  public Node(int x, int y, Node parentNode){
      this.x=x;
      this.y=y;
      this.parentNode=parentNode;
  }
  
  public int getX() {
      return x;
  }
  public void setX(int x) {
      this.x = x;
  }
  public int getY() {
      return y;
  }
  public void setY(int y) {
      this.y = y;
  }
  public Node getParentNode() {
      return parentNode;
  }
  public void setParentNode(Node parentNode) {
      this.parentNode = parentNode;
  }
  public double getG() {
      return g;
  }
  public void setG(double g) {
      this.g = g;
  }
  public double getH() {
      return h;
  }
  public void setH(double h) {
      this.h = h;
  }
  public double getF() {
      return f;
  }
  public void setF(double f) {
      this.f = f;
  }
}
