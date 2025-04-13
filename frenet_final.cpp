#include <bits/stdc++.h>

struct Point {
    double x, y;
    Point(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
};

class Model {
public:
    double x, y, theta;
    Model(double x0, double y0, double theta0) : x(x0), y(y0), theta(theta0) {}

    void update(double v, double omega, double dt) {
        x += v * cos(theta) * dt;
        y += v * sin(theta) * dt;
        theta += omega * dt;
        while (theta > M_PI) theta -= 2 * M_PI;
        while (theta < -M_PI) theta += 2 * M_PI;
    }
};

class Path {
    public:
        std::vector<Point> waypoints;
        std::vector<double> s_values;
    
        Path() {
            // generates waypoints for the shape 8
            for (double t = 0.0; t <= 2 * M_PI; t += 0.05) {  // t can be adjusted for more/less points
                double x = sin(t);
                double y = sin(t) * cos(t);
                waypoints.emplace_back(x, y);
            }
    
            // calculates the corresponding s values for each waypoint
            double s = 0.0;
            s_values.push_back(s);  // Starting s value at 0
            for (size_t i = 1; i < waypoints.size(); ++i) {
                double dx = waypoints[i].x - waypoints[i-1].x;
                double dy = waypoints[i].y - waypoints[i-1].y;
                s += sqrt(dx * dx + dy * dy);  // summation of linear distances between consecutive points
                s_values.push_back(s);
            }
        }
    
        std::pair<double, double> CarttoFrenet(const Point& p, size_t& currentPos) const {
            double min_dist = std::numeric_limits<double>::max();
            size_t best_k = currentPos; 
            for (size_t k = 0; k < waypoints.size(); ++k) {
                Point a = waypoints[k];
                Point b = waypoints[(k + 1) % waypoints.size()]; //enclosing the loop
                Point ab_vec{b.x - a.x, b.y - a.y};
                Point ap{p.x - a.x, p.y - a.y};
                double ab_len = sqrt(ab_vec.x * ab_vec.x + ab_vec.y * ab_vec.y);
                if (ab_len < 1e-10) continue;
                double t = std::max(0.0, std::min(1.0, (ap.x * ab_vec.x + ap.y * ab_vec.y) / (ab_len * ab_len)));
                Point closest = {a.x + t * ab_vec.x, a.y + t * ab_vec.y};
                double dist = sqrt(pow(p.x - closest.x, 2) + pow(p.y - closest.y, 2));
                if (dist < min_dist) {
                    min_dist = dist;
                    best_k = k;
                }
            }
            currentPos = best_k;
            Point a = waypoints[best_k];
            Point b = waypoints[(best_k + 1) % waypoints.size()];
            Point ab_vec{b.x - a.x, b.y - a.y};
            Point ap{p.x - a.x, p.y - a.y};
            double ab_len = sqrt(ab_vec.x * ab_vec.x + ab_vec.y * ab_vec.y);
            if (ab_len < 1e-10) ab_len = 1e-10;
            double t = std::max(0.0, std::min(1.0, (ap.x * ab_vec.x + ap.y * ab_vec.y) / (ab_len * ab_len)));
            double s = s_values[best_k] + t * (s_values[(best_k + 1) % s_values.size()] - s_values[best_k]);
            s = fmod(s, s_values.back());
            double d = sqrt(pow(p.x - (a.x + t * ab_vec.x), 2) + pow(p.y - (a.y + t * ab_vec.y), 2));
            double cross = ab_vec.x * (p.y - a.y) - ab_vec.y * (p.x - a.x);
            if (cross < 0) d = -d;
            return {s, d};
        }
    
        double FindAngle(double s) const {
            s = fmod(s, s_values.back());
            size_t k = 0;
            while (k < s_values.size() - 1 && s_values[k + 1] < s) ++k;
            if (k == s_values.size() - 1) k = 0; // enclosing the loop
            Point a = waypoints[k];
            Point b = waypoints[(k + 1) % waypoints.size()];
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double segment_length = sqrt(dx * dx + dy * dy);
            if (segment_length < 1e-5) return 0.0;
            double t = (s - s_values[k]) / (s_values[(k + 1) % s_values.size()] - s_values[k]);
            return atan2(dy, dx);
        }
    
        Point FrenettoCart(double s) const {
            s = fmod(s, s_values.back());
            size_t k = 0;
            while (k < s_values.size() - 1 && s_values[k + 1] < s) ++k;
            double t = (s - s_values[k]) / (s_values[(k + 1) % s_values.size()] - s_values[k]);
            Point a = waypoints[k];
            Point b = waypoints[(k + 1) % waypoints.size()];
            double x = a.x + t * (b.x - a.x);
            double y = a.y + t * (b.y - a.y);
            return Point{x, y};
        }
    };

class Obstacle {
public:
    double s_obs;
    double v_obs;
    Path* path;

    Obstacle(double initial_s, double velocity, Path* p)
        : s_obs(initial_s), v_obs(velocity), path(p) {}

    void update(double dt) {
        // tried writing a function to update the dynamic position of the obstacle
        // need to debug this part
        // as of now we consider the obstacle to be static
    }

    Point getPosition() const {
        return path->FrenettoCart(s_obs);
    }
};

double CostFunc(const Model& model, const Path& path, std::vector<Obstacle>& obstacles,
                  double s, double d, double omega, double dt, int horizon_steps) {
    double total_cost = 0.0;
    Model temp_model(model.x, model.y, model.theta);
    size_t currentPos = 0;
    auto [init_s, init_d] = path.CarttoFrenet({temp_model.x, temp_model.y}, currentPos);

    for (int i = 0; i < horizon_steps; ++i) {
        double theta_path = path.FindAngle(s);
        double path_error = 500.0 * d * d; 
        double obs_penalty = 0.0;
        for (const auto& obs : obstacles) {
            Point obs_pos = obs.getPosition();
            double dist_to_obs = sqrt(pow(temp_model.x - obs_pos.x, 2) + pow(temp_model.y - obs_pos.y, 2));
            obs_penalty += (dist_to_obs < 0.25) ? 1.0 / (6 * dist_to_obs + 0.01) : 0.0;
        }
        double control_effort = 0.15 * omega * omega;

        total_cost += path_error + obs_penalty + control_effort;

        temp_model.update(0.05, omega, dt);
        auto [new_s, new_d] = path.CarttoFrenet({temp_model.x, temp_model.y}, currentPos);
        s = new_s;
        d = new_d;
    }
    return total_cost;
}

int main() {

    // initializing model and generating path
    const double dt = 0.1;
    const double total_time = 230.0;
    const double v = 0.03;
    Path path;
    double initial_theta = path.FindAngle(0.0);
    Model model(path.waypoints[0].x, path.waypoints[0].y, initial_theta);
    size_t currentPos = 0;


    // defining obstacles
    std::vector<Obstacle> obstacles;
    obstacles.emplace_back(0.380791, 0.0, &path); 
    obstacles.emplace_back(1.4568, 0.0, &path); 
    obstacles.emplace_back(3.5498, 0.0, &path); 
    const int horizon_steps = 10;


    // writing output to a file for later visualization
    std::ofstream outfile("output_final.txt");
    if (!outfile.is_open()) {
        std::cerr << "Error opening output.txt for writing!" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < path.waypoints.size(); ++i) {
        outfile << "W," << path.s_values[i] << "," << path.waypoints[i].x << "," << path.waypoints[i].y << "\n";
    }
    for (size_t i = 0; i < obstacles.size(); ++i) {
        Point initial_obstacle = obstacles[i].getPosition();
        outfile << "O" << i << "," << obstacles[i].s_obs << "," << initial_obstacle.x << "," << initial_obstacle.y << "\n";
    }


    // motion planning loop
    for (double t = 0.0; t < total_time; t += dt) {
        auto [s, d] = path.CarttoFrenet({model.x, model.y}, currentPos);
        double best_omega = 0.0;
        double min_cost = std::numeric_limits<double>::max();

        for (double omega = -0.5; omega <= 0.5; omega += 0.05) {  // limit omega to a range
            double cost = CostFunc(model, path, obstacles, s, d, omega, dt, horizon_steps);
            if (cost < min_cost) {
                min_cost = cost;
                best_omega = omega;
            }
        }

        model.update(v, best_omega, dt);

        //writing the state of the robot to the output file
        outfile << "T," << t << "," << model.x << "," << model.y << "," << model.theta << ","
                << s << "," << d << "," << best_omega << ",";
        for (const auto& obs : obstacles) {
            outfile << obs.s_obs << ",";
        }
        outfile << "\n";
    }

    outfile.close();
    std::cout << "motion executed successfully" << std::endl;
    return 0;
}