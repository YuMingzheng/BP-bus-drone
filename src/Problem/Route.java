package Problem;

import java.util.ArrayList;
import java.util.Collections;

/**
 * @author Yu Mingzheng
 * @date 2024/8/29 16:25
 * @description
 */
public class Route implements Cloneable {
    public double distance, Q;    // Q表示该路径在列生成过程中是否被“选择”，即对应的变量y_i是否不等于0
    public ArrayList<Integer> path;
    public ArrayList<Integer> throughOrder;

    public Route() {
        this.path = new ArrayList<>();
        this.distance = 0.0;
        throughOrder = new ArrayList<>();
    }

    public Route clone() throws CloneNotSupportedException {
        Route route = (Route) super.clone();
        route.path = (ArrayList<Integer>) path.clone();
        route.throughOrder = (ArrayList<Integer>) throughOrder.clone();
        return route;
    }

    public void addCity(int city) {
        this.path.add(city);
    }

    public void setDistance(double c) {
        this.distance = c;
    }

    public double getDistance() {
        return this.distance;
    }

    public void setQ(double a) {
        this.Q = a;
    }

    public double getQ() {
        return this.Q;
    }

    public ArrayList<Integer> getPath() {
        return this.path;
    }

    public void reversePath() {
        Collections.reverse(path);
    }

    public String toString(){
        return "Route@[served_order: " + throughOrder.toString() + ", distance: " + distance + ", path: " + path.toString() + "]";

//        return "【Path : " + path.toString() + " , Cost : " + cost + "】\n";
    }
}
