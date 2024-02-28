#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;

class Node1 : public rclcpp::Node
{
public:
  Node1() : rclcpp::Node("node1")
  {
    service_ = create_service<std_srvs::srv::Trigger>(
      "node1/trigger", std::bind(&Node1::srv_cb, this, _1, _2, _3));
    client_ = create_client<std_srvs::srv::Trigger>("node2/trigger");
  }
private:
  void srv_cb(std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
              const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> request)
  {
    auto async_cb = [service, request_header, request](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      (void)request;
      std_srvs::srv::Trigger::Response response;
      response.message = "Node 2 said: '" + future.get()->message + "'";
      service->send_response(*request_header, response);
    };

    auto request_inner = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_->async_send_request(request_inner, async_cb);
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

class Node2 : public rclcpp::Node
{
public:
  Node2() : rclcpp::Node("node2")
  {
    service_ = create_service<std_srvs::srv::Trigger>(
      "node2/trigger", std::bind(&Node2::srv_cb, this, _1, _2));
  }
private:
  void srv_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    response->message = "Hello!";
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node1 = std::make_shared<Node1>();
  auto node2 = std::make_shared<Node2>();
  executor.add_node(node1);
  executor.add_node(node2);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
