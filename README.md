# Demo of calling services from within callbacks

A major pain point of ROS 2 is not being able to call services from within any sort of callback.
This is caused by the executor being occupied by the callback, and not free to process service responses, causing a deadlock.

Many forum posts suggest using multiple callback groups to solve this problem, but this involves creating
multiple threads, which is not always desirable. It also means your node will require a MultiThreadedExecutor.

This solution provides a way to use a SingleThreadedExecutor, and still be able to call services from within callbacks. Only one executor thread is used, and it is not blocked by the callback.

It utilizes asynchronous service calls and **deferred service responses**, a feature available since ROS 2 Humble.

In this demo, NodeA and NodeB are created, and added to a SingleThreadedExecutor, and spun. Nothing special.

```cpp
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
```

Node1 provides a service of type `std_srvs/srv/Trigger` on `node1/trigger`.
It also creates a client to call a service of type `std_srvs/srv/Trigger` on `node2/trigger`.

```cpp
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
    ...
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};
```

NodeB provides a service of type `std_srvs/srv/Trigger` on `node2/trigger`, which simply responds with a message saying ``"Hello!"``.

```cpp
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
```

Now, going back to Node1's srv_cb function, this function's desired behavior is to call the ``node2/trigger`` service, prepend the response's message with ``"Node 2 said: "``, and send that back.

To do this, we create an asynchronous callback function, and call the second service with ``async_send_request`` function. This allows the executor to finish srv_cb and free up without returning a response. The executor can then listen to the service response from Node2. When the service response is received, the ``async_cb`` lambda is called, which sends a deferred response back to the original service call.

Note the callback signature is different to the ones in the basic service examples. This is because we are using a deferred response.

```cpp
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
```

To test this, run the program:
```
ros2 run nested_services_rclcpp_demo nested_service
```

and call the service from the command line:

```bash
ros2 service call /node1/trigger std_srvs/srv/Trigger
```

You should expect a response:

```bash
 ros2 service call /node1/trigger std_srvs/srv/Trigger
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message="Node 2 said: 'Hello!'")
```
