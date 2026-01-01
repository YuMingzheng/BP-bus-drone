package IO;

/**
 * @author Yu Mingzheng
 * @date 2024/12/27 15:57
 * @description
 */
public class ModelInputData {
    double edge_distance ;
    double depot_2_i;
    double depot_2_j;
    double i_time_l;
    double i_time_r;
    double j_time_l;
    double j_time_r;
    int edge_type;
    int has_customer;

    public ModelInputData(double edge_distance,
                          double depot_2_i,
                          double depot_2_j,
                          double i_time_l,
                          double i_time_r,
                          double j_time_l,
                          double j_time_r,
                          int edge_type,
                          int has_customer
    ) {
        this.edge_distance = edge_distance;
        this.depot_2_i = depot_2_i;
        this.depot_2_j = depot_2_j;
        this.i_time_l = i_time_l;
        this.i_time_r = i_time_r;
        this.j_time_l = j_time_l;
        this.j_time_r = j_time_r;
        this.edge_type = edge_type;
        this.has_customer = has_customer;
    }

    public ModelInputData() {

    }

    @Override
    public String toString() {
        return "[" + edge_distance + ", "
                + depot_2_i + ", "
                + depot_2_j + ", "
                + i_time_l + ", "
                + i_time_r + ", "
                + j_time_l + ", "
                + j_time_r + ", "
                + edge_type + ", "
                + has_customer + "]";
    }

    public void setEdge_distance(double edge_distance) {
        this.edge_distance = edge_distance;
    }

    public void setDepot_2_i(double depot_2_i) {
        this.depot_2_i = depot_2_i;
    }

    public void setDepot_2_j(double depot_2_j) {
        this.depot_2_j = depot_2_j;
    }

    public void setI_time_l(double i_time_l) {
        this.i_time_l = i_time_l;
    }

    public void setI_time_r(double i_time_r) {
        this.i_time_r = i_time_r;
    }

    public void setJ_time_l(double j_time_l) {
        this.j_time_l = j_time_l;
    }

    public void setJ_time_r(double j_time_r) {
        this.j_time_r = j_time_r;
    }

    public void setEdge_type(int edge_type) {
        this.edge_type = edge_type;
    }

    public void setHas_customer(int has_customer) {
        this.has_customer = has_customer;
    }
}
