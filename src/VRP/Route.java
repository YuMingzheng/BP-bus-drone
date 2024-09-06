package VRP;

import java.util.ArrayList;
import java.util.Collections;

/**
 * @author Yu Mingzheng
 * @date 2024/8/29 16:25
 * @description
 */
public class Route implements Cloneable {
    public double cost, Q;    // Q表示该路径在列生成过程中是否被“选择”，即对应的变量y_i是否不等于0
    public ArrayList<Integer> path;

    public Route() {
        this.path = new ArrayList<>();
        this.cost = 0.0;
    }

    public Route clone() throws CloneNotSupportedException {
        Route route = (Route) super.clone();
        route.path = (ArrayList<Integer>) path.clone();
        return route;
    }

    public void addCity(int city) {
        this.path.add(city);
    }

    public void setCost(double c) {
        this.cost = c;
    }

    public double getCost() {
        return this.cost;
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
        return "【Path : " + path.toString() + " , Cost : " + cost + "】\n";
    }
}
