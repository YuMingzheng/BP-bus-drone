package Old;

import java.util.ArrayList;
import java.util.Collections;

/**
 * @author Yu Mingzheng
 * @date 2024/9/3 15:40
 * @description
 */
public class Label {
    final int vertexId;
    final double cost;
    final double time;
    final double demand;

    final Label preLabel;

    public Label(double cost, double time, double demand, int vertexId, Label preLabel) {
        this.cost = cost;
        this.time = time;
        this.demand = demand;
        this.vertexId = vertexId;
        this.preLabel = preLabel;
    }

    public Label(double cost, double time, double demand, int vertexId){
        this.cost = cost;
        this.time = time;
        this.demand = demand;
        this.vertexId = vertexId;
        this.preLabel = null;
    }

    public boolean dominate(Label that){
        // TODO 对于到达终点的label，只比较cost
//        if(this.vertexId  == Parameters.numClient + 1){
//            if (this.cost >= that.cost)
//                return false;
//            return true;
//        }
        if(this.demand > that.demand || this.cost > that.cost || this.time > that.time)
            return false;
        return true;
//        if(this.demand <= that.demand && this.cost <=  that.cost && this.time <= that.time)
//            return true;
//        else return false;
    }

    public ArrayList<Integer> getVisitVertexes(){
        ArrayList<Integer> vertexIds = new ArrayList<>();
        vertexIds.add(vertexId);

        Label label = this;
        while ((label = label.preLabel) != null){
            vertexIds.add(label.vertexId);
        }
        Collections.reverse(vertexIds);
        return vertexIds;
    }

    public boolean equals(Object other){
        if (other == this)
            return true;

        if (!(other instanceof Label))
            return false;

        Label label = this;
        Label that = (Label) other;
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

    public boolean visitedNode(int index){
        Label label = this;
        while ((label = label.preLabel) != null){
            if(label.vertexId == index) return true;
        }
        return false;
    }

    public String toString(){
        ArrayList<Integer> vertexIds = getVisitVertexes();

        StringBuilder sb = new StringBuilder();
        vertexIds.forEach(id -> sb.append(id).append("-"));
        sb.deleteCharAt(sb.length() - 1);
        sb.append(String.format("【cost: %f, time: %f, demand: %f】\n", cost, time, demand));

        return sb.toString();
    }
}