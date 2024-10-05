package Problem.Node;

/**
 * @author Yu Mingzheng
 * @date 2024/9/11 16:58
 * @description
 */
public class TimeNode extends Node {
    public double arrivalTime;
    public double departTime;

    public TimeNode(double[] node, double[] time) {
        super(node[0], node[1]);
        this.arrivalTime = Math.round(time[0] * 100.0) / 100.0;
        this.departTime = Math.round(time[1] * 100.0) / 100.0;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof TimeNode) {
            TimeNode value = (TimeNode) obj;
            return (this.x == value.x) && (this.y == value.y) && (this.arrivalTime == value.arrivalTime) && (this.departTime == value.departTime);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(x) ^ Double.hashCode(y) ^ Double.hashCode(arrivalTime) ^ Double.hashCode(departTime);
    }

    @Override
    public String toString() {
        return "[(" + x + "," + y + "),(" + arrivalTime + "," + departTime + ")]";
    }

    public double[] getTimeWindow() {
        return new double[]{arrivalTime, departTime};
    }
}
