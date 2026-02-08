/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <champ/utils/urdf_loader.h>
#include <boost/algorithm/string.hpp>
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

class ContactSensor: public rclcpp::Node
{
	public:
		ContactSensor():
			foot_contacts_ {false,false,false,false},
			Node("contacts_sensor",rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true))
		{
			std::vector<std::string> joint_names;

			joint_names = champ::URDF::getLinkNames(this->get_node_parameters_interface());
			foot_links_.push_back(joint_names[2]);
			foot_links_.push_back(joint_names[6]);
			foot_links_.push_back(joint_names[10]);
			foot_links_.push_back(joint_names[14]);

			// Initialize pubs and subs
			gz_contacts_subscriber_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
				"/foot_contact_gz", 10,
				std::bind(&ContactSensor::gzCallback_, this, std::placeholders::_1));
			contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

		}

		// void gzCallback_(const gz::msgs::Contacts &_msg)
		void gzCallback_(const ros_gz_interfaces::msg::Contacts &_msg)
		{
			for(size_t i = 0; i < 4; i++)
			{
				foot_contacts_[i] = false;
			}

			for (int i = 0; i < _msg.contacts.size(); ++i) 
			{
				std::vector<std::string> results;

				//! Adapted std::string to gz::msgs::Entity
				auto contact = _msg.contacts[i];
				std::string collision = contact.collision2.name;
				if (collision.empty()) {
					// Handle case where name is not populated (optional debugging)
					RCLCPP_WARN(this->get_logger(), "Collision1 name is empty for contact %d", i);
					continue;
				}
				// std::string collision = _msg.contact(i).collision1();

				boost::split(results, collision, [](char c){return c == ':';});
				// Ensure results has enough elements
				if (results.size() < 3) {
							RCLCPP_WARN(this->get_logger(), "Unexpected collision name format: %s", collision.c_str());
							continue;
				}

				for(size_t j = 0; j < 4; j++)
				{
					if(foot_links_[j] == results[2])
					{
						foot_contacts_[j] = true;
						break;
					}
				}
			}

		}

		void publishContacts()	
		{
			champ_msgs::msg::ContactsStamped contacts_msg;
			contacts_msg.header.stamp = this->get_clock()->now();
			contacts_msg.contacts.resize(4);

			for(size_t i = 0; i < 4; i++)
			{
				contacts_msg.contacts[i] = foot_contacts_[i];
			}
			
			contacts_publisher_->publish(contacts_msg);
		}
	
	private:
		bool foot_contacts_[4];
		std::vector<std::string> foot_links_;
		rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
		rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr gz_contacts_subscriber_;
};

void exitHandler(int sig)
{
	// gazebo::client::shutdown();
	rclcpp::shutdown();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ContactSensor>();
	rclcpp::Rate loop_rate(50);

	while (rclcpp::ok())
	{
		node->publishContacts();
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}