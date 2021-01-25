#include <ros/ros.h>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <iostream>

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

typedef server::message_ptr message_ptr;

void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
    std::cout << "on_message called with hdl: " << hdl.lock().get()
              << " and message: " << msg->get_payload()
              << std::endl;
    if (msg->get_payload() == "stop-listening") {
        s->stop_listening();
        return;
    }

    try {
        s->send(hdl, msg->get_payload(), msg->get_opcode());
    } catch (websocketpp::exception const & e) {
        std::cout << "Echo failed because: "
                  << "(" << e.what() << ")" << std::endl;
    }
}
int main(int argc, char** argv)
{
  ros::init(argc,argv,"data_pub_node");
  ros::NodeHandle nh;
  server echo_server;
  try {
    echo_server.set_access_channels(websocketpp::log::alevel::all);   echo_server.clear_access_channels(websocketpp::log::alevel::frame_payload);
    echo_server.init_asio(); 		  echo_server.set_message_handler(bind(&on_message,&echo_server,::_1,::_2));
    echo_server.listen(9002);
    echo_server.start_accept();
    echo_server.run();
    } catch (websocketpp::exception const & e) {
        std::cout << e.what() << std::endl;
    } catch (...) {
        std::cout << "other exception" << std::endl;
    }
}
