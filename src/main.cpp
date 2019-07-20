#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

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

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  bool twiddle = false;
  if (twiddle)
  {
    pid.Init(0.0, 0.0, 0.0);
  } else
  {
    pid.Init(0.416933, 0.0, 2.03224);
  }    
  double tol = 0.2;
  int twiddle_count = 0;
  int fwd_stps = 0;
  double best_error = 10000000.0;
  double err = 0.0;
  int max_n = 2000;
  double throttle_value = 0.0;

  h.onMessage([&pid, &twiddle, &tol, &twiddle_count, &fwd_stps, &best_error, &err, &max_n, &throttle_value](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
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
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          bool reset = false;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if(twiddle)
          {
            //use twiddle to fine tune the PID controller
            switch (twiddle_count % 6)
            {
              case 0: if(fwd_stps == 0)
                      {
                        err = 0.0;
                        err += pow(cte,2);
                        pid.KpInc();
                      } else 
                      {
                        err += pow(cte,2);
                        if((err > best_error) && (fwd_stps < max_n))
                        {
                          reset = true;
                        }
                        if((err < best_error) && (fwd_stps == max_n))
                        {
                          best_error = err;
                          pid.dKpInc();
                          twiddle_count += 1;
                        }            
                      }
                      break;
              case 1: if(fwd_stps == 0)
                      {
                        err = 0.0;
                        err += pow(cte,2);
                        pid.KpDec();
                      } else
                      {
                        err += pow(cte,2);
                        if((err > best_error) && (fwd_stps <= max_n))
                        {
                          reset = true;
                          pid.KpInc();
                          pid.dKpDec();
                        }
                        if((err < best_error) && (fwd_stps == max_n))
                        {
                          best_error = err;
                          pid.dKpInc();
                        }                       
                      }
                      break;
              case 2: if(fwd_stps == 0)
                      {
                        err = 0.0;
                        err += pow(cte,2);
                        pid.KdInc();
                      } else 
                      {
                        err += pow(cte,2);
                        if(err > best_error)
                        {
                          reset = true;
                        }
                        if((err < best_error) && (fwd_stps == max_n))
                        {
                          best_error = err;
                          pid.dKdInc();
                          twiddle_count += 1;
                        }            
                      }
                      break;
              case 3: if(fwd_stps == 0)
                      {
                        err = 0.0;
                        err += pow(cte,2);
                        pid.KdDec();
                      } else
                      {
                        err += pow(cte,2);
                        if((err > best_error) && (fwd_stps <= max_n))
                        {
                          reset = true;
                          pid.KdInc();
                          pid.dKdDec();
                        }
                        if((err < best_error) && (fwd_stps == max_n))
                        {
                          best_error = err;
                          pid.dKdInc();
                        }
                        //twiddle = pid.ResetTwiddle(tol);                       
                      }
                      break;
              case 4: if(fwd_stps == 0)
                      {
                        err = 0.0;
                        err += pow(cte,2);
                        pid.KiInc();
                      } else 
                      {
                        err += pow(cte,2);
                        if(err > best_error)
                        {
                          reset = true;
                        }
                        if((err < best_error) && (fwd_stps == max_n))
                        {
                          best_error = err;
                          pid.dKiInc();
                          twiddle_count += 1;
                        }            
                      }
                      break;
              case 5: if(fwd_stps == 0)
                      {
                        err = 0.0;
                        err += pow(cte,2);
                        pid.KiDec();
                      } else
                      {
                        err += pow(cte,2);
                        if((err > best_error) && (fwd_stps <= max_n))
                        {
                          reset = true;
                          pid.KiInc();
                          pid.dKiDec();
                          twiddle = pid.ResetTwiddle(tol);
                        }
                        if((err < best_error) && (fwd_stps == max_n))
                        {
                          best_error = err;
                          pid.dKiInc();
                          twiddle = pid.ResetTwiddle(tol);
                        }                
                      }
                      break;            
            }
            pid.UpdateError(cte);
            steer_value = pid.TotalError();          
          } else
          {
            //drive the car use the PID controller
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            
          }
          // max steer angle = 20 deg
          double steer_angle = rad2deg(steer_value);
          if(steer_angle > 20.0) {
            steer_angle = 20.0; 
          }else if (steer_angle < -20.0) {
            steer_angle = -20.0;
          }
          steer_value = deg2rad(steer_angle);
          // throttle control
          throttle_value = 0.3;          
          if((speed > 20.0) && (abs(steer_angle) > 10.0)) 
          {
            throttle_value = -0.1;
          } 
          // DEBUG
          if((fwd_stps==0) | (fwd_stps==max_n)|1) 
          {
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                      << " Throttle Value: " << throttle_value << std::endl;
            std::cout << "Twiddle = " << twiddle << " iteration " << twiddle_count 
                      << " fwd steps = " << fwd_stps << " error = " << err
                      << " best error = " << best_error << std::endl;
            pid.piddebug();
          }
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          string msg;
          if( (twiddle && fwd_stps == max_n) || reset ) {
            msg = "42[\"reset\",{}]";
            twiddle_count += 1;
            fwd_stps = 0;
            pid.ResetError();
          } else {
            msg = "42[\"steer\"," + msgJson.dump() + "]";
            fwd_stps += 1;
          }          
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