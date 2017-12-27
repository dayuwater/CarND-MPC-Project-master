#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}



// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);
    
    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }
    
    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

int main() {
    uWS::Hub h;
    
    // MPC is initialized here!
    MPC mpc;
    double previous_steer=0;
    double previous_throttle=0;
    
    h.onMessage([&mpc, &previous_steer, &previous_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                cout << "In:" << s << endl;
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    
                    // current state
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    double steering = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];
                    
                    
                    json msgJson;
                    
                    
                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    // use this for waypoint overplots
                    
                    // TODO:
                    // Because the data returned from the server are in global coordinate
                    // We need to transform those coordinate to car's coordinate
                    // 1. Implement transmission
                    // 2. Test if the car position is correct by overplotting on the simulator
                    // 3. See what the waypoints returned from the server actually means
                    // 4. Select only the waypoints that are in the driving direction
                    
                    // Results:
                    // The waypoint returned from ther server is actually the waypoint
                    // It covers the center of the lane
                    
                    Eigen::VectorXd eptsx(ptsx.size());
                    Eigen::VectorXd eptsy(ptsy.size());
                    
                    
                    for(int i=0; i<ptsx.size(); i++){
                        double ptx = ptsx[i] - px;
                        double pty = ptsy[i] - py;
                        
                        double tx = ptx*cos(psi) + pty*sin(psi);
                        double ty = -ptx*sin(psi) + pty*cos(psi);
                        
                        // add the transformed waypoints to the vector to draw reference line
                        next_x_vals.push_back(tx);
                        next_y_vals.push_back(ty);
                    
                        // add the transformed waypoints to the Eigen vector to fit polynomial for MPC
                        eptsx[i] = tx;
                        eptsy[i] = ty;
                
                    }
                    
                   
                    
                    
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                   
                    
                    /*
                     * TODO: Calculate steering angle and throttle using MPC.
                     *
                     * Both are in between [-1, 1].
                     *
                     *
                     */
                    double steer_value;
                    double throttle_value;

                
                    // the polynomial to be fit should be independent to delay
                    auto coeffs = polyfit(eptsx, eptsy, 3);
                    
                   
                    // change the direction of the steering because the simulator uses an inverted angle scheme
                    steering = steering * -1;
                    
                    // determine the car's state at t=100ms using kinematic model
                    // although there is a processing and communication delay, it is neglectable on my computer
                    // but this might be a problem on VM enviornments
                    double delay = 0.1; // increase this if using VM
                    double projectedX = delay * v;
                    // y is still 0 at t=100ms
                    double projectedPsi = v/2.67 * steering * delay;
                    double projectedV = v + throttle * delay;
                    
                    // calculate the cross track error
                    // the current state of x and y should be 0
                    double cte = polyeval(coeffs, 0) - 0 + (v * sin(0 - atan(coeffs[1]))* delay);
                    // calculate the orientation error
                    // because the car is at (0,0), the terms with x should be 0, only the 1st order term is kept
                    double epsi = 0 - atan(coeffs[1]) +  v/2.67 * steering * delay;
                    
                    Eigen::VectorXd state(6);
                    state << projectedX, 0, projectedPsi, projectedV, cte, epsi;

                    auto solution = mpc.Solve(state, coeffs);
                    
                    
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    
                   
                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;
                    
                   
                    int N = 25;
                    mpc_x_vals.resize(N);
                    mpc_y_vals.resize(N);

                    // the results from MPC are in car's coordinate, no need to transform
                    for(int i=0; i<N; i++){
                        double ptx = solution[i+2];
                        double pty = solution[i+N+2];
                        mpc_x_vals[i] = ptx;
                        mpc_y_vals[i] = pty;
                    }
                    
                    
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    
                    steer_value = solution[0] / deg2rad(25) * -1;
                    throttle_value = solution[1];
                    
                    
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
                    
                
                    // record the steer and throttle value for next iteration
                    previous_steer = steer_value;
                    previous_throttle = throttle_value;
                    
                    
                    
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;
                    
                 
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
