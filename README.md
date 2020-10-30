---
description: >-
  Notes/cheat sheet from following roboticsbackend.com ROS2 course
  (https://www.udemy.com/course/ros2-for-beginners/)
---

# ROS2 Basics

## Creating a Workspace

In home dir 

```text
mkdir ros2_ws
cd ros2_ws/
mkdir src
colcon build
```

Creates sub folders within workspace.

In `install` folder there is a `setup.bash` script. This needs sourcing \(in a similar way to ros2 needs sourcing in terminal\).

{% hint style="info" %}
For new setup; install colcon common extensions and source the auto complete bash script

```text
sudo apt install python3-colcon-common-extensions
```
{% endhint %}

## Creating a Package

To create a node\(program\) you need to create a package to contain it- you may have one for a camera and one for motion planning for example. Each is an independent, and reusable block. A package may have multiple nodes. e.g. Camera pkg has driver node & image processing node. \(The processing could be sperate if it was not specific to a camera\).

### Python

Navigate to `ros2_ws/src` then 

```text
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

 creates a package called my\_py\_pkg that is a python package that depends on the ros python library 'rclpy' \(which is needed for all ros python packages.\)

Add nodes to folder `src\my_py_pkg\my_py_pkg`

### C++

```text
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

{% hint style="info" %}
For both Python and C++ - To build: cd to `ros2_ws` and execute  `colcon build` for all packages or `colcon build --packages-select my_pkg` to compile just that package
{% endhint %}

## Nodes

{% hint style="info" %}
A subpart of an application with a single purpose.
{% endhint %}

Which communicate with each other through topics, services, and parameters

### Python Node Template 

{% tabs %}
{% tab title="OOP Template" %}
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
{% endtab %}

{% tab title="OOP Commented" %}
```python
#!/usr/bin/env python3 
import rclpy #required to use ros2 functionalities
from rclpy.node import Node

#name class what the node will do e.g. CameraDriverNode
class MyNode(Node):

    def __init__ (self): 
        super().__init__("py_test")
        self.get_logger().info("Hello ROS2") #code to do stuff here e.g log

def main(args=None):
    rclpy.init(args=args) #required in all ros2 nodes to init ros2 communication
    node = MyNode() #create node
    rclpy.spin(node) #keeps looping here, so callbacks can be called
    rclpy.shutdown() #tidy up at end of node

if __name__ == "__main__":
    main()
```
{% endtab %}

{% tab title="Basic" %}
{% hint style="warning" %}
Not recommended use OOP for better scalability
{% endhint %}

```python
#!/usr/bin/env python3 
import rclpy #required to use ros2 functionalities
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args) #required in all ros2 nodes to init ros2 communication
    node = Node("py_test") #create and name node, best practice not to include 'node' in node name
    node.get_logger().info("Hello ROS2") #code to do stuff (in this case log)
    rclpy.spin(node) #keeps looping here, so callbacks can be called
    rclpy.shutdown() #at end of node

if __name__ == "__main__":
    main()
```
{% endtab %}
{% endtabs %}

Need to install file into workspace. In `setup.py` 

```python
entry_points={
        'console_scripts': [
            "py_node = my_py_pkg.my_first_node:main"
        ],
    },
```

Then `colcon build`

Can then run with `ros2 run my_py_pkg py_node`

{% hint style="warning" %}
3 different places where you name the node! Can be different - \(Often the same\)

File name - can be different from... 

Class name \(created in line 7 of OOP template above\) - \[used in code\] - can be different from...

Node name \(created in line 9 of OOP template above\) - \[Displayed in logs when printing to term\] -  can be different from...

Executable name \(created in setup.py when installing node in workspace\) - \[Used to run node\].
{% endhint %}

### C++ Node Template

{% tabs %}
{% tab title="OOP Template" %}
```cpp
#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
{% endtab %}

{% tab title="OOP example" %}
```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node ??");

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);               //init ros comms
    auto node = std::make_shared<MyNode>(); //now this main function won't chance as everything added to the class
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
{% endtab %}

{% tab title="Basic" %}
```cpp
    rclcpp::init(argc, argv); //init ros comms
    auto node = std::make_shared<rclcpp::Node>("cpp_test"); //create and name node using shared pointer
    RCLCPP_INFO(node->get_logger(), "Hello Cpp Node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
{% endtab %}

{% tab title="CMakeLists.txt" %}
```cpp
cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
```
{% endtab %}
{% endtabs %}

In VS Code you need to tell it where to find rclcpp. Hover over `#include "rclcpp/rclcpp.hpp"` , press `ctrl+shift+p` and type C/C++ Edit Configurations \(JSON\) and add `/opt/ros/foxy/include` to the `includePath.`

Compile using cmakelist.txt. He removed Default to c99 and tests for min example. Then add code to create executable and install it. 

Now compile using `colcon build` this creates an executable in the lib folder, but also can be run directly from ros: `ros2 run my_cpp_pkg cpp_node`

{% hint style="warning" %}
Note similar naming issues as above
{% endhint %}

### Build

Must be in Ros2 Workspace

```python
cd ros2_ws
```

Can build all using

```python
colcon build
```

or select a package using

```python
colcon build --packages-select my_pkg_name
```

For Python packages you can link colcon build so that it doesn't need to be compiled every time. The first time you compile use this command:

```python
colcon build --packages-select my_py_pkg --symlink-install
```

{% hint style="warning" %}
The file must be an executable or the symlink will fail.  

Use:`chmod +x my_node.py`
{% endhint %}

### Debug Tools

```python
# see all nodes that are running
ros2 node list 

# see info about a node
ros2 node info node_name
```

### RQT

RQT is a collection of plugins with a graphical interface, run simply with `rqt`

`rqt_graph` to directly get node graph \(or go through menu `plugings > introspection > node graph`\)

## Topics

A named message bus that can be published to and subscribed from. 

### Python example:

{% tabs %}
{% tab title="Publisher" %}
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")

        self.robot_name_ = "C3PO"
        self.publisher_ = self.create_publisher(String, "robot_news", 10) 
        #create publisher with msg type, topic name, queue size
        self.timer = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Station has been started")

    def publish_news(self): #good practice to make function to publish to topic
        msg = String()
        msg.data = "Hi this is " +str(self.robot_name_) + " from robot news station"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
{% endtab %}

{% tab title="Subscriber" %}
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartphoneNode(Node): 
    def __init__(self):
        super().__init__("smartphone")
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Smartphone been started")

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```
{% endtab %}
{% endtabs %}

### C++ Example

{% tabs %}
{% tab title="Publisher" %}
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station Has Started");

    }

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi this is ") + robot_name_ + std::string(" from the Robot News Station");
        publisher_->publish(msg);
    }

    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
{% endtab %}

{% tab title="Subscriber" %}
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smartphone")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_news", 10,
            std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smarpohone has been started.");
    }

private:
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
{% endtab %}
{% endtabs %}

### Debug Tools

Get topics and info

```bash
$ ros2 topic list
$ ros2 topic info /topic_name
```

Info shows message type. Use command below to see the data type

```text
$ ros2 interface show type/name
```

Create subscriber in terminal

```text
$ ros2 topic echo /topic_name
```

Find publishing frequency and bandwidth

```text
$ ros2 topic hz /topic_name
$ ros2 topic bw /node_name
```

Publish ad-hoc to topic from terminal

```text
$ ros2 topic pub -r 10 /topic_name type/name "{data: Hello from Terminal}"
```

## Rename at runtime

{% hint style="danger" %}
It is possible to run two nodes with the same name - Do not do this it will cause conflicts!
{% endhint %}

Instead, rename the **node** at runtime using:

```text
$ ros2 run my_py_pkg py_node --ros-args --remap __node:=new_name
```

or `-r` instead of `--remap`

**Topics** can also be renamed at runtime - same command for publisher and subscriber nodes

```text
$ ros2 run my_pkg my_node --ros-args -r topic_name:=new_name
```

**Services** can also be remapped at runtime in the same way:

```text
$ ros2 run my_pkg my_node --ros-args -r service_name:=new_name
```

## Services

Services are used for client/server interaction between nodes.

### Server Example

{% tabs %}
{% tab title="Python" %}
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server_ = self.create_service(
            AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add Two Ints Server has been started")

    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```
{% endtab %}

{% tab title="C++" %}
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Server has been started.");
    }

private:
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d + %d", request->a, request->b, response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

```
{% endtab %}
{% endtabs %}



### Client Example

`client.call` is an available object for a synchronous call to a server, but it is blocking and in some situations may become stuck so recommended at always use `client.call_async`

Non-oop examples run once and then shutdown but help show the concept.

{% tabs %}
{% tab title="Python no OOP" %}
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_oop")

    client = node.create_client(AddTwoInts, "add_two_ints")
    while not client.wait_for_service(1):
        node.get_logger().warn("Waiting for Server Add Two Ints...")
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8

    # future is filled once server responds
    future = client.call_async(request)
    # waits until future object is complete, but allows other callback to execute
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(str(request.a) + " + " +
                               str(request.b) + " = " + str(response.sum))
    except Exception as e:
        node.get_logger().error("Service call failed %r" % (e,))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
{% endtab %}

{% tab title="Python OOP" %}
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6, 7)
        self.get_logger().info("Add client been started")

    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Add Two Ints...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_add_two_ints, a=a, b=b))

    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " +
                                   str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```
{% endtab %}

{% tab title="C++ no OOP" %}
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop"); 

    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while(!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for server...");
    }

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 3;
    request->b = 8;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "%d + %d = %d", request->a, request->b, future.get()->sum);
    }
    else{
        RCLCPP_ERROR(node->get_logger(), "Error while calling service");
    }



    rclcpp::shutdown();
    return 0;
}
```
{% endtab %}

{% tab title="C++ OOP" %}
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        //dont call like this: callAddTwoIntsService(1, 4);
        //thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4)); //call like this in new thread so not blocking
        //for multiple calls neew to use a thread pool - different ways depending on application and how to manage threads
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 33, 4)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 12)));
    }

    void callAddTwoIntsService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);
        //waiting for future blocks thread so have to call function in new thread, or it never gets to 'spin' line so wouldnt get response
        try
        {
            auto response = future.get(); //waiting on future here
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    //std::thread thread1_; //single thread
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
{% endtab %}
{% endtabs %}

### Debug Tools

List services running:

```text
$ ros2 service list
```

Call service without creating a node:

```text
$ ros2 service call /service_name service_type "{a:testdata, b:testdata}"
```

Or for a graphical interface run `rqt` and click `plugins > Services > Service Caller`

Find interface type for a service:

```text
$ ros2 service type /service_name
```

Show the request and response format of the service type

```text
$ ros2 interface show service/type/name
```

## Custom Interfaces

Messages defined by a Name and a data type

Primitive data types that can be used are listed: 

{% embed url="https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/" %}

Common ones, maybe installed already

{% embed url="https://github.com/ros2/common\_interfaces" %}

Can have a msg inside a msg definition so dont have to redefine them but can reuse instead.

### Make custom interfaces pkg

Topics and Services are the communication layer. Interfaces are the actual content that you send.

To recap, here’s how to create a custom interface:

* Create a new package only for your msg and srv definitions.
* Setup the package \(CMakeLists.txt and package.xml\)
* Create a msg/ and srv/ folders, place your custom msg definitions and srv definitions here.

Once you’ve setup your package, adding a new interface is really simple:

* Add a new file in the right folder: msg/ or srv/
* Add one line into CMakeLists.txt
* Compile with “colcon build”
* And don’t forget to source your ROS2 workspace when you want to use those messages!

Here’s what you can use inside a msg or srv definition:

* Any primitive type defined by ROS2 \(most common ones: int64, float64, bool, string, and array of those\)
* Any message you’ve already created in this package.
* Any message from another package. In this case don’t forget to add a dependency for the other package in both package.xml and CMakeLists.txt.

And now, when you compile the definitions, new interfaces will be created, along with headers/modules ready to be included in your C++ or Python nodes.

```text
$ cd ros2_ws/src
$ ros2 pkg create my_robot_interfaces
$ cd my_robot_interfaces
$ rm -rf include/
$ rm -rf src/
$ mkdir msg
```

Add these 3 lines to package.xml:

```markup
 <build_depend>rosidl_default_generators</build_depend>
 <excec_depend>rosidl_default_runtime</exec_depend>
 <member_of_group>rosidl_interface_packages</member_of_group>
```

In CMakeLists.txt add the line under `# find dependencies`:

```markup
find_package(rosidl_default_generators REQUIRED)
```

Under this add the following:

```markup
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsgDefiniton.msg"
)
```

To make a msg, always name using CamelCase with no 'msg/message' in the name.

```markup
$ cd msg
$ touch CustomMsgDefiniton.msg
```

It must then be built:

```markup
$ colcon build --packages-select my_robot_interfaces
```

To add more messages most of this setup is not needed again as it was for the package. Make a new .msg file and in CMakeLists.txt add its name under rosidle\_generate\_interfaces.... line.

### Custom Service

Make srv dir in custom\_interfaces pkg and add .srv files here.

All .srv definition files have this format

```markup
request
---
repsonse1
response2
etc.
```

The .srv interface also needs to be added to the CMakesLists.txt as above. It can then be built. 

Can check it has worked by using: 

```text
$ ros2 interface show my_robot_interaces/
```

ros2 interface has some other options which maybe useful for debug, tab twice to see.

To drill down and find out what data is needed you can use these commands

```text
ros2 node list
ros2 topic list
ros2 topic info
ros2 interface show
ros2 node info
ros2 service list
ros2 service type 
```



## Parameters

These are settings within a node, that can be changed at runtime.

Can work with these directly from the terminal with these commands:

```text
$ ros2 param
delete describe sump get list set

$ros2 param get /node_name parametername
gives the value here
```

Inside a node you can declare and get parameters:

{% tabs %}
{% tab title="Python" %}
```python
self.declare_parameter("param_name", defaultValue)
self.variable_ = self.get_parameter("param_name").value
```
{% endtab %}

{% tab title="C++" %}
```cpp
this->declare_parameter("param_name", defaultValue);
number_ = this->get_parameter("param_name").as_int(); 
// have to cast to right type so change 'as_int' bit to right type
// eg
// string_ = this->get_parameter("param_name).as_string();
```
{% endtab %}
{% endtabs %}

To set a parameter:

```text
$ ros2 run my_py_pkg node_name --ros-args -p parametername:=value
```

## Launch Files

Allows setting of parameter, renaming of nodes at runtime and running of many nodes without having to do it all from the terminal each time. 

They can be created in any package but it is better to create a new package, often called robotname\_bringup.

```text
$ cd ros2_ws/src/
$ ros2 pkg create my_robot_bringup
$ cd my_robot_bringup
$ rm -rf include/
$ rm -fr src/
$ mkdir launch
```

In CmakeLists.txt remove the 'Default to C99' section and the 'If\(BUILD\_TESTING\)' section and add;

```text
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

launch files are made in the launch folder and are python files named; application\_name.launch.py. Make it executable using chmod +x

### Template

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher"
        name="new_node_name"
        remappings=[
            ("topic_name", "new_topic_name"),
            ("service_name", "new_service_name")
        ]
        parameters=[
            {"parameter_name": value},
            {"other param_name": value}
        ]
    )

    ld.add_action(number_publisher_node)
    return ld
```

Can launch from py or cpp packages.

Add the dependency to package.xml;

```text
<exec_deped>my_py_pkg</exec_deped>
```

To use the file;

```text
$ ros2 launch my_robot_bringup number_app.launch.py
```

To change parameters, add the below inside Node\(\), these could be declared at the top and used in multiple nodes eg if two publish to the same renamed topic.

```python
name="new_node_name"
remappings=[
        ("topic_name", "new_topic_name"),
        ("service_name", "new_service_name")
]
parameters=[
        {"parameter_name": value},
        {"other param_name": value}
]
```

Recap:

Setup for launch files:

* Create a new package &lt;robot\_name&gt;\_bringup \(best practice\).
* Create a launch/ folder at the root of the package.
* Configure CMakeLists.txt to install files from this launch/ folder.
* Create any number of files you want inside the launch/ folder, ending with .launch.py.

Run a launch file:

* After you’ve written your file, use “colcon build” to install the file.
* Don’t forget to source your environment
* Start the launch file with “ros2 launch &lt;package&gt; &lt;name\_of\_the\_file&gt;

