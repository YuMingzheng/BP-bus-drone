package Algorithm.Labeling;

import Parameters.ExtendGraph;

import java.util.ArrayList;
import java.util.Collections;

/**
 * @author Yu Mingzheng
 * @date 2024/9/8 16:50
 * @description
 */
public class MyLabel {
    int vertexId;        // 访问的最后一个点
    double cost;         // 累计的reduced cost
    int isLoad;          // 离开点i时是否载货，取值为0或1
    double duration;     // 离开点i时的剩余飞行距离
    double arrivalTime;  // 到达时间
    double serviceTime;  // 服务开始时间
    boolean[] openReq;   // 所有Request在点i处是否Open（一个Request状态为Open，指该订单被pick但未被delivery）
    boolean[] servedReq; // 所有Request在点i处是否已经被服务过

    MyLabel preLabel;

    public MyLabel(int vertexId, double cost, int isLoad, double duration, double arrivalTime, double serviceTime, boolean[] openReq, boolean[] servedReq, MyLabel preLabel) {
        this.vertexId = vertexId;
        this.cost = cost;
        this.isLoad = isLoad;
        this.duration = duration;
        this.arrivalTime = arrivalTime;
        this.serviceTime = serviceTime;
        this.openReq = openReq;
        this.servedReq = servedReq;
        this.preLabel = preLabel;
    }

    public MyLabel(int vertexId, double cost, int isLoad, double duration, double arrivalTime, double serviceTime, boolean[] openReq, boolean[] servedReq) {
        this.vertexId = vertexId;
        this.cost = cost;
        this.isLoad = isLoad;
        this.duration = duration;
        this.arrivalTime = arrivalTime;
        this.serviceTime = serviceTime;
        this.openReq = openReq;
        this.servedReq = servedReq;
    }

    public boolean dominate(MyLabel that){
        // TODO 对于到达终点的label，只比较cost
//        if(that.vertexId == ExtendGraph.nodeNumExtendStatic-1)
//            return false;

        if(this.cost <= that.cost
                && this.duration >= that.duration
                && this.serviceTime <= that.serviceTime
                && MyLabel.isSubset(this.servedReq , that.servedReq)
                && MyLabel.areEqual(this.openReq , that.openReq)
        ) return true;
        else return false;
    }

    /**
     * 判断集合A是否是集合B的子集，这里的集合使用boolean数组来进行表示的
     * @param A 集合A
     * @param B 集合B
     * @return true or false
     */
    public static boolean isSubset(boolean[] A, boolean[] B) {
        for (int i = 0; i < A.length; i++) {
            if (A[i] && !B[i]) {
                return false;
            }
        }
        return true;
    }

    /**
     * 判断集合A是否与集合B相等
     * @param A 集合A
     * @param B 集合B
     * @return true or false
     */
    public static boolean areEqual(boolean[] A, boolean[] B) {
        if (A.length != B.length) {
            return false;
        }
        for (int i = 0; i < A.length; i++) {
            if (A[i] != B[i]) {
                return false;
            }
        }
        return true;
    }

    /**
     * 获取当前label所访问的节点列表
     * @return ArrayList 访问的节点列表
     */
    public ArrayList<Integer> getVisitVertexes(){
        ArrayList<Integer> vertexIds = new ArrayList<>();
        vertexIds.add(vertexId);

        MyLabel label = this;
        while ((label = label.preLabel) != null){
            vertexIds.add(label.vertexId);
        }
        Collections.reverse(vertexIds);
        return vertexIds;
    }

    /**
     * 判断this 和 that 是否相同
     * @param other that label
     * @return boolean
     */
    public boolean equals(Object other){
        if (other == this)
            return true;

        if (!(other instanceof MyLabel))
            return false;

        MyLabel label = this;
        MyLabel that = (MyLabel) other;
        if (that.vertexId != label.vertexId)
            return false;

        while ((label = label.preLabel) != null) {
            that = that.preLabel;
            if (that == null)
                return false;
            if (that.vertexId != label.vertexId)
                return false;
        }

        if (that.preLabel != null)
            return false;
        return true;
    }

    /**
     * 判断当前label是否走过了index节点
     * @param index 要进行判断的节点
     * @return boolean
     */
    public boolean visitedNode(int index){
        MyLabel label = this;
        while ((label = label.preLabel) != null) {
            if(label.vertexId == index){ return true; }
        }
        return false;
    }

    public String toString(){
        ArrayList<Integer> vertexIds = getVisitVertexes();

        StringBuilder sb = new StringBuilder();
        vertexIds.forEach(id -> sb.append(id).append("-"));
        sb.deleteCharAt(sb.length()-1);
        sb.append(String.format("【cost: %f, serviceTime: %f, duration: %f】\n", cost, serviceTime, duration));

        return sb.toString();
    }




}
