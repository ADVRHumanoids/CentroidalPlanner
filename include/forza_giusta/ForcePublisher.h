#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>
#include <centroidal_planner/vectorStringLinks.h>
#include <std_srvs/SetBool.h>

namespace force_publisher {
    
  class ForcePublisher
  {
    
  public:
      
      ForcePublisher(std::vector<std::string> contact_links);

      void send_force(const Eigen::VectorXd& f_opt);
      void send_force(std::vector<std::string> contact_links, const Eigen::VectorXd &f_opt);
      void send_normal(const Eigen::VectorXd& n_opt);
//      void send_wrench_manip(const Eigen::VectorXd& tau_manip);
//      void send_force_arms(const Eigen::VectorXd& f_arms);
      void setLiftedContacts(std::vector<std::string> lifted_links);
      void setContacts(std::vector<std::string> contact_links);
      void setPointContacts(std::vector<std::string> point_contact_links);

      void switch_controller(bool flag);


  private:
      
      std::map<std::string, ros::Publisher> _pubs_force;
//      std::vector<ros::Publisher> _pubs_force;

//      std::map<std::string, ros::Publisher> _pubs_force_arms;
//      std::vector<ros::Publisher> _pubs_force_arms;

      std::map<std::string, ros::Publisher> _pubs_normal;
//      std::vector<ros::Publisher> _pubs_normal;

//      ros::Publisher _pub_wrench_manip;
      std::vector<std::string> _contact_links;
//      std::vector<std::string> _pushing_links;

      ros::ServiceClient _lift_client, _contact_client, _point_contact_client;
      ros::ServiceClient _switch_client;
  };

  ForcePublisher::ForcePublisher(std::vector<std::string> contact_links)
  {
      ros::NodeHandle nh;
      _contact_links = contact_links;

      for(auto l : contact_links)
      {
          _pubs_force[l] = nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/force_ref/" + l, 1);

          while (_pubs_force[l].getNumSubscribers() == 0)
          {
              ros::Duration(0.01).sleep();
              ros::spinOnce();
          }

          _pubs_normal[l] = nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/normal/" + l, 1);

          while (_pubs_normal[l].getNumSubscribers() == 0)
          {
              ros::Duration(0.01).sleep();
              ros::spinOnce();
          }
      }
      
      _switch_client = nh.serviceClient<std_srvs::SetBool>("forza_giusta/switch_controller/");

      _lift_client = nh.serviceClient<centroidal_planner::vectorStringLinks>("forza_giusta/set_lifted_contacts");

      _contact_client = nh.serviceClient<centroidal_planner::vectorStringLinks>("forza_giusta/set_contacts");

      _point_contact_client = nh.serviceClient<centroidal_planner::vectorStringLinks>("forza_giusta/set_point_contacts");



//      _pub_wrench_manip = nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/wrench_manip/", 1);
      
//      std::vector<std::string> pushing_links  = {""};

//      for(auto i : pushing_links)
//      {
//          _pubs_force_arms[i] = nh.advertise<geometry_msgs::WrenchStamped>("forza_giusta/force_arms/" + i, 1);

//          while (_pubs_force_arms[i].getNumSubscribers() == 0)
//          {
//              ros::Duration(0.01).sleep();
//              ros::spinOnce();
//          }
//      }
      
  }

//  void ForcePublisher::send_force(const Eigen::VectorXd &f_opt)
//  {
//      for (int i : {0, 1, 2, 3})
//      {
//          Eigen::Vector3d f =  f_opt.segment<3>(3*i);

//          geometry_msgs::WrenchStamped msg;
//          msg.header.frame_id = "world";
//          msg.header.stamp = ros::Time::now();
//          msg.wrench.force.x = f.x();
//          msg.wrench.force.y = f.y();
//          msg.wrench.force.z = f.z();

//          _pubs_force[i].publish(msg);
//      }

//  }

  void ForcePublisher::send_force(const Eigen::VectorXd &f_opt)
  {
      int i = 0;
      for (auto l : _contact_links)
      {
          Eigen::Vector3d f =  f_opt.segment<3>(3*i);
	      
          geometry_msgs::WrenchStamped msg;
          msg.header.frame_id = "world";
          msg.header.stamp = ros::Time::now();
          msg.wrench.force.x = f.x();
          msg.wrench.force.y = f.y();
          msg.wrench.force.z = f.z();

          _pubs_force[l].publish(msg);

          i++;
	  
      }
      
  }

  void ForcePublisher::send_force(std::vector<std::string> contact_links, const Eigen::VectorXd &f_opt)
  {
      int i = 0;
      for (auto l : contact_links)
      {
          Eigen::Vector3d f =  f_opt.segment<3>(3*i);

          geometry_msgs::WrenchStamped msg;
          msg.header.frame_id = "world";
          msg.header.stamp = ros::Time::now();
          msg.wrench.force.x = f.x();
          msg.wrench.force.y = f.y();
          msg.wrench.force.z = f.z();

          _pubs_force[l].publish(msg);

          i++;

      }

  }

//  void ForcePublisher::send_wrench_manip(const Eigen::VectorXd &tau_manip)
//  {
      	      
//	  geometry_msgs::WrenchStamped msg;
//	  msg.header.frame_id = "world";
//	  msg.header.stamp = ros::Time::now();
//	  msg.wrench.force.x = tau_manip[0];
//	  msg.wrench.force.y = tau_manip[1];
//	  msg.wrench.force.z = tau_manip[2];
//	  msg.wrench.torque.x = tau_manip[3];
//	  msg.wrench.torque.y = tau_manip[4];
//	  msg.wrench.torque.z = tau_manip[5];
	
//      _pub_wrench_manip.publish(msg);
	  
      
//  }
  
  void ForcePublisher::send_normal(const Eigen::VectorXd &n_opt)
  {
      
      int i = 0;
      for (auto l : _contact_links)
      {
          Eigen::Vector3d n =  n_opt.segment<3>(3*i);

          geometry_msgs::WrenchStamped msg;
          msg.header.frame_id = "world";
          msg.header.stamp = ros::Time::now();
          msg.wrench.force.x = n.x();
          msg.wrench.force.y = n.y();
          msg.wrench.force.z = n.z();

          _pubs_normal[l].publish(msg);
	  
      }
      
  }


  void ForcePublisher::setLiftedContacts(std::vector<std::string> lifted_links)
  {
      centroidal_planner::vectorStringLinks srv;

      for (auto l : lifted_links)
      {
           srv.request.link.push_back(l);
      }

      for (auto l : srv.request.link)
      {
          std::cout << l << std::endl;
      }

      if (_lift_client.call(srv))
      {
          ROS_INFO("Success to call service 'set_lifted_contacts' for links: \n");
          for (auto elem : lifted_links)
          {
              ROS_INFO(elem.c_str());
          }
      }
      else
      {
          ROS_INFO("Failed to call service 'set_lifted_contacts'");
      }

  }

  void ForcePublisher::setContacts(std::vector<std::string> contact_links)
  {
      centroidal_planner::vectorStringLinks srv;
      for (auto l : contact_links)
      {
          srv.request.link.push_back(l);
      }

      if (_contact_client.call(srv))
      {
          ROS_INFO("Success to call service 'set_contacts' for links: \n");
          for (auto elem : contact_links)
          {
              ROS_INFO(elem.c_str());
          }
      }
      else
      {
          ROS_INFO("Failed to call service 'set_contacts'");
      }

  }

  void ForcePublisher::setPointContacts(std::vector<std::string> point_contact_links)
  {
      centroidal_planner::vectorStringLinks srv;
      for (auto l : point_contact_links)
      {
          srv.request.link.push_back(l);
      }

      if (_point_contact_client.call(srv))
      {
          ROS_INFO("Success to call service 'set_point_contact_links' for links: \n");
          for (auto elem : point_contact_links)
          {
              ROS_INFO(elem.c_str());
          }
      }
      else
      {
          ROS_INFO("Failed to call service 'set_point_contact_links'");
      }

  }
  void ForcePublisher::switch_controller(bool flag)
  {
      std_srvs::SetBool srv;
      srv.request.data = flag;

      if (_switch_client.call(srv))
      {
          ROS_INFO("Success to call service 'switch_controller'");
      }
      else
      {
          ROS_ERROR("Failed to call service 'switch_controller'");
      }
  }
//  void ForcePublisher::send_force_arms(const Eigen::VectorXd &f_arms)
//  {
      
//      int i = 0;
//      for (auto l : _pushing_links)
//      {

//          Eigen::Vector3d f =  f_arms.segment<3>(3*i);

//          geometry_msgs::WrenchStamped msg;
//          msg.header.frame_id = "world";
//          msg.header.stamp = ros::Time::now();
//          msg.wrench.force.x = f.x();
//          msg.wrench.force.y = f.y();
//          msg.wrench.force.z = f.z();

//          _pubs_force_arms[l].publish(msg);
	  
//      }
      
//}

}

