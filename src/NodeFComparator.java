import java.util.Comparator;

/**
 * 节点比较
 */
class NodeFComparator implements Comparator<Node> {

    @Override
    public int compare(Node o1, Node o2) {
        if (o1.getF() - o2.getF() > 0) {
            return 1;
        } else if (o1.getF() - o2.getF() < 0) {
            return -1;
        } else {
            return 0;
        }
    }

}