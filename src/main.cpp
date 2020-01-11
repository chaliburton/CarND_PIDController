#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <fstream>
#include <vector>
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
	  
	  
// double runtest(double kp, double ki,double kd) {
int main(int argc, char *argv[]) {
	//system ("C:\Users\Chris\Documents\Udacity\Project8\term2_sim_windows\term2_sim_windows\term2_sim.exe");
	
	uWS::Hub h;

	PID pid;							// Create pid for lateral steering control
	PID vpid;							// Create pid for velocity target
	vpid.Init(-1.4,-0.001,-0.1);				// initialize velocity pid with high proportional gain
	double init_Kp = atof(argv[1]);  				// input Kp to initialize steering control
	double init_Ki = atof(argv[2]);					// input Ki to initialize steering control
	double init_Kd = atof(argv[3]);					// input Kd to initialize steering control
	std::vector<double> p = {init_Kp, init_Ki, init_Kd};	    	// store in vector for passing by reference
	
	double d_init = std::max(std::max(p[0],p[1]),p[2])*0.1;
	std::vector<double> dp = { d_init , d_init , d_init }; 	//store deltap params for TWIDDLE
	pid.Init(init_Kp, init_Ki, init_Kd);				// initialize steering pid

	/*
	 * initialize variables for TWIDDLE control
	 */
	int i = 0;
	int jj = 0;
	int k = 0;
	double CTE_n = 0;
	double best_err = 999999;
	double v_error;	

	h.onMessage([&vpid, &v_error, &pid, &p, &dp, &i, &jj, &k, &best_err, &CTE_n](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) { //needed to pass dp and p vectors // delete CTE_n?	    // "42" at the start of the message means there's a websocket message event.
	    // The 4 signifies a websocket message
	    // The 2 signifies a websocket event
	    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
	      auto s = hasData(string(data).substr(0, length));

	      if (s != "") {
		auto j = json::parse(s);

		string event = j[0].get<string>();

		if (event == "telemetry") {
		  // j[1] is the data JSON object
		  double cte = std::stod(j[1]["cte"].get<string>());
		  double speed = std::stod(j[1]["speed"].get<string>());
		  double angle = std::stod(j[1]["steering_angle"].get<string>());
		  double steer_value;
		  double speed_control;
		  double spd_tgt = 10;
		  double prev_CTE = CTE_n;
		/*
		 * Count N loops then implement twiddle for each N messages
		 * Track error since beginning of TWIDDLE run to compare
		 */
		  i++;
		  CTE_n += cte*cte;
		  double deltaCTE = 
		 /*
		  * Calculate speed target based on the steering angle 
		  * (this has issues with steering bias/misalignment off center)
		  * need to reduce speed target shen steering inputs results in potential loss of control
		  */
		  if(cte>2) {
			spd_tgt=3;
		  } else if (speed<10) {
			spd_tgt = 10;
		  } else if ( (std::abs(angle))>20.0) {
			spd_tgt = 10.0;
		  } else if ( (std::abs(angle))>15.0) {
			spd_tgt = 10.0;
		  } else if ( (std::abs(angle))>10.0) { 
			spd_tgt = 15.0;
		  } else if ( (std::abs(angle))> 3.0) {
		        spd_tgt = 40.0;
		  } else if ( (std::abs(angle))> 1.50) {
		        spd_tgt = 70.0;
		  } else {     
			spd_tgt = 100.0;
		  } 
		  double spd_err = spd_tgt - speed;

		 /*
		  * Update error for velocity pid & update control output for speed
		  */
		  vpid.UpdateError(spd_err);
		  speed_control = vpid.TotalError();
		 /*
		  * Update steering error & update control output for steering 
		  */
		  pid.UpdateError(cte);
		  steer_value = pid.TotalError();

		 /* 
		  * IMPLEMENT TWIDDLE HERE
		  */			
		  if(i>4500){
		       /*
			* Reset the simulator and iterations for TWIDDLE
			* Intoduce TWIDDLE tolerance
			*/
			string msg = "42[\"reset\",{}]";
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			i = 0;
			double tol = 0.0045; 

			/*
			 * IMPLEMENTING TWIDDLE CORE HERE
			 */
			std::cout<<std::endl<< tol << " Current: "<<(dp[0]+dp[1]+dp[2]) <<std::endl;
			if ( (dp[0]+dp[1]+dp[2]) > tol) {  // the PID still needs tuning
				if(k==0){
					p[jj] += dp[jj];
					std::cout<<"1st: k = "<<k<<"      jj = "<<jj<<"p[jj] = "<<p[jj]<<std::endl;
					if(CTE_n<best_err){
						best_err = CTE_n;
					}
				}
				if(CTE_n<best_err && k == 1){  		// if the result is better move it on the next pass by incrementing dp by 10% and store the best error
					best_err = CTE_n;
					dp[jj] *=1.1;
					std::cout<<"2nd: k = "<<k<<"      jj = "<<jj<<std::endl;
				} else if(k == 1) {		
					p[jj] -= 2*dp[jj];
					std::cout<<"2nd: k = "<<k<<"      jj = "<<jj<<std::endl;
				} 
				if(CTE_n<best_err && k == 2){
					best_err = CTE_n;
					dp[jj] *= 1.1;	
					jj = (jj+1)%3; 
					k = -1;
					std::cout<<"3rd: k = "<<k<<"      jj = "<<jj<<std::endl;
				} else if(k == 2){
					p[jj] += dp[jj];
					dp[jj] *= 0.9;
					jj = (jj+1)%3;
					k = -1;
					std::cout<<"3rd: k = "<<k<<"      jj = "<<jj<<std::endl;
				}
				k++;
				/*
				 * Output current value of PID controller parameters, reinitialize with TWIDDLE
				 * Output new values for next TWIDDLE run
				 */
				std::cout<<"OLD PID = " <<pid.GetP()<<" "<<pid.GetI()<<" "<<pid.GetD()<<" CTE_n now: "<<CTE_n<<"  CTE_Best = "<<best_err<<std::endl;
				pid.Init(p[0], p[1], p[2]);
				std::cout<<"NEW PID = " <<pid.GetP()<<" "<<pid.GetI()<<" "<<pid.GetD()<<std::endl;
				CTE_n = 0;
			} else {
				/* 
				 * When TWIDDLE is complete, pause runs to record values
				 */
				do {		
					std::cout << '\n' << "TWIDDLE Complete, record PID values & press a key to continue...";
	      			} while (std::cin.get() != '\n');
				std::cout<<"End test runs run"<<std::endl;
			}
		  }
		  // TWIDDLE ENDS HERE

		  json msgJson;
		  msgJson["steering_angle"] = steer_value;
		  msgJson["throttle"] = speed_control;
		  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
		  //std::cout << msg << std::endl;
	  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}  // end "telemetry" if
	      } else {
		// Manual driving
		string msg = "42[\"manual\",{}]";
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	      }
	    }  // end websocket message if
	  }); // end h.onMessage

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
