#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>

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

