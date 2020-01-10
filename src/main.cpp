#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <fstream>
#include <vector>
//#include <Windows.h>

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

	PID pid;
	double init_Kp = atof(argv[1]);  //can clean this assignment up
	double init_Ki = atof(argv[2]);
	double init_Kd = atof(argv[3]);
	std::vector<double> p = {init_Kp, init_Ki, init_Kd};
	std::vector<double> dp = {0.2, 0.1, 0.1};
	pid.Init(init_Kp, init_Ki, init_Kd);
	int i = 0;
	int j = 0;
	int k = 0;
	double CTE_n = 0;
	double best_err = 999999;

	h.onMessage([&pid, &p, &dp, &i, &j, &k, &best_err, &CTE_n](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) { //needed to pass dp and p vectors // delete CTE_n?	    // "42" at the start of the message means there's a websocket message event.
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
		  /**
		   * TODO: Calculate steering value here, remember the steering value is
		   *   [-1, 1].
		   * NOTE: Feel free to play around with the throttle and speed.
		   *   Maybe use another PID controller to control the speed!
		   */


		  pid.UpdateError(cte);
		  steer_value = pid.TotalError();

		  // DEBUG
		  std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
			    << std::endl;

		  /*
		   * Count N loops then implement twiddle for each N messages
		   */
		  i++;
		  std::cout<<i<<"th iteration"<<std::endl;
		  CTE_n += cte;
		  if(i>400){
			
			//kp_twid, ki_twid, kd_twid = pid.twiddle(CTE_n);		//need to determine where to calculate total error, also where to keep best error?  do we have multiple PIDs or reinitilize and have another global most likely
			//pid.Init(kp_twid,ki_twid,kd_twid); 			//update from twiddle and reset error terms
			std::cout<<"kp update: " << pid.GetP() <<" ki update: "<<pid.GetI()<<" ki update: "<<pid.GetD<<std::endl<<std::endl;
			//system("taskkill /IM term2_sim.exe /F");
			//myfile.close();
			//run_complete = 0;
			//json msgJson;
			string msg = "42[\"reset\",{}]";
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			i = 0;
			double tol = 0.1; // tolerance for when to stop TWIDDLE
			//std::count << msg << std::endl;
			//IMPLEMENTING TWIDDLE HERE//
			if ( (dp[0]+dp[1]+dp[2]) > tol) {  // the PID still needs tuning
				if(k==0){
					p[j] += dp[j];
					k++;
				}
				if(k==1 || k==2){
					if(CTE_n<best_err){  		// if the result is better move it on the next pass by incrementing dp by 10% and store the best error
						best_err = CTE_n;
						dp[j] *=1.1;
						k++;
					} else if(k == 1) {		
						p[j] -= 2*dp[j];
						k++;
					} else {
						p[j] += dp[j];
						dp[j] *= 0.9;	
						k++;
					}
				} 
				if(k>1) {
					k = 0;
					j++;
				}
			}
		  
		/*} else we are finished tuning the PID
					  //reset simulator here in tol, tolerance loop
			} else { ( (dp[0]+dp[1]+dp[2]) > tol) { //move this check uptop?
			std::cout<<"End test runs run"<<std::endl;
			//keep going logic?
			//reset and run a complete lap
			}*/
			/// modified till here
		  }

		  json msgJson;
		  msgJson["steering_angle"] = steer_value;
		  msgJson["throttle"] = 0.2;
		  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
		  std::cout << msg << std::endl;
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
	  

/*
int main(int argc, char *argv[]) {
  double init_Kp = atof(argv[1]);
  double init_Ki = atof(argv[2]);
  double init_Kd = atof(argv[3]);
  int i = 0;
  double tol = 1;
  std::vector<double> p = {init_Kp, init_Ki, init_Kd};
  std::vector<double> dp = {1, 1, 1};
  while( (dp[0]+dp[1]+dp[2]) > tol) {
	do 
	{
	   std::cout << '\n' << "Press a key to continue...";
	} while (std::cin.get() != '\n');
	std::cout<<"First run"<<std::endl;
        double best_err = runtest(p[0], p[1], p[2]);
	std::cout<<best_err<<std::endl;
	for(int j = 0; j<p.size(); j++) {
		
	  p[j] += dp[j];
	  do 
	  {
	     std::cout << '\n' << "Press a key to continue...";
	  } while (std::cin.get() != '\n');
	  double new_err = runtest(p[0], p[1], p[2]);
	  if(new_err < best_err) {
	      best_err = new_err;
	      dp[j] *=1.1;
	  } else {
	      p[j] -= 2*dp[j];
	      do 
	      {
		std::cout << '\n' << "Press a key to continue...";
	      } while (std::cin.get() != '\n');

		  
		  
		  
	      new_err = runtest(p[0], p[1], p[2]);
	      if( new_err < best_err) {
		  best_err = new_err;
		  dp[j] *=1.1;
	      } else {
		  p[j] += dp[j];
		  dp[j] *= 0.9;
	      }
	  }
	}
  }
}
*/
