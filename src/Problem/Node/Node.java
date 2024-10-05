package Problem.Node;

/**
 * @author Yu Mingzheng
 * @date 2024/9/11 16:53
 * @description
 */
public class Node {
    public double x;
    public double y;

    public Node(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Node(String nodeStr) {
        // 解析字符串，如 "Node(40, 40)"
        String content = nodeStr.substring(nodeStr.indexOf('(') + 1, nodeStr.indexOf(')'));
        String[] parts = content.split(",");
        this.x = Double.parseDouble(parts[0].trim());
        this.y = Double.parseDouble(parts[1].trim());
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Node) {
            Node value = (Node) obj;
            return (this.x == value.x) && (this.y == value.y);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(x) ^ Double.hashCode(y);
    }

    @Override
    public String toString() {
        return "(" + x + "," + y + ")";
    }

    public double get(int key) {
        if (key == 0) {
            return x;
        } else if (key == 1) {
            return y;
        } else {
            throw new IndexOutOfBoundsException();
        }
    }

    public double[] getLocation() {
        return new double[]{x, y};
    }

    public boolean greaterThan(Node other) {
        return (this.x > other.x) || (this.x == other.x && this.y >= other.y);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
