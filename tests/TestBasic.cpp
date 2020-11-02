#include <gtest/gtest.h>
#include <CentroidalPlanner/CentroidalPlanner.h>
#include <CentroidalPlanner/CoMPlanner.h>

class TestBasic: public ::testing::Test {
    

protected:

     TestBasic(){
         
     }

     virtual ~TestBasic() {
     }

     virtual void SetUp() {
         
     }

     virtual void TearDown() {
     }
     
     
};


TEST_F(TestBasic, testSimpleProblem)
{
    
    double robot_mass = 100.0;
    double g = -9.81;
    
    std::vector<std::string> contact_name;
    contact_name.push_back("contact1");
    
    double ground_z = 0.1;
    auto ground_env = std::make_shared<cpl::env::Ground>();
    ground_env->SetGroundZ(ground_z);
    
    auto cpl = std::make_shared<cpl::CentroidalPlanner>(contact_name, robot_mass, ground_env);
    
    cpl::solver::Solution sol;
    int status = cpl->Solve(sol);
    
    std::cout<< "sol:" << sol << std::endl;
    std::cout<< "status:" << status << std::endl;
    
    double Fz_tot = 0.0;
    
    for (auto& elem: sol.contact_values_map)
    {       
        Fz_tot += elem.second.force_value.z();  
        
        EXPECT_NEAR(elem.second.position_value.z(), ground_z, 1e-6);
        EXPECT_NEAR(elem.second.normal_value.norm(), 1.0, 1e-6);
        EXPECT_NEAR(elem.second.normal_value.z(), 1.0, 1e-6);
        
    }

    EXPECT_NEAR(Fz_tot, -robot_mass*g, 1e-6);
    
};


TEST_F(TestBasic, testGroundEnv)
{
    
    double robot_mass = 100.0;
    double g = -9.81;
    
    std::vector<std::string> contact_name;
    contact_name.push_back("contact1");
    contact_name.push_back("contact2");
    contact_name.push_back("contact3");
    contact_name.push_back("contact4");
    
    double ground_z = 0.1;
    auto ground_env = std::make_shared<cpl::env::Ground>();
    ground_env->SetGroundZ(ground_z);
    double mu = 0.5;
    ground_env->SetMu(mu);
    
    auto cpl = std::make_shared<cpl::CentroidalPlanner>(contact_name, robot_mass, ground_env);
    cpl->SetCoMWeight(2.0);
    cpl->SetForceWeight(0.0);
    
    Eigen::Vector3d p_lb, p_ub;
    p_lb  << -0.3, -0.3, 0.0;
    p_ub  <<  0.3,  0.3, 1.0;

    cpl->SetPosBounds("contact1",p_lb,p_ub);
    cpl->SetPosBounds("contact2",p_lb,p_ub);
    cpl->SetPosBounds("contact3",p_lb,p_ub);
    cpl->SetPosBounds("contact4",p_lb,p_ub);
    
    Eigen::VectorXd manip_wrench;
    manip_wrench.setZero(6);
    manip_wrench[0] = 100.0;
    manip_wrench[5] = 100.0;
    cpl->SetManipulationWrench(manip_wrench);
    
    cpl::solver::Solution sol;
    int status = cpl->Solve(sol);

    std::cout<< "sol:" << sol << std::endl;
    std::cout<< "status:" << status << std::endl;

    Eigen::Vector3d F_sum, Torque_sum; 
    F_sum.setZero();
    Torque_sum.setZero();
    
    for (auto& elem: sol.contact_values_map)
    {       
        F_sum += elem.second.force_value;   
        Torque_sum += (elem.second.position_value - sol.com_sol).cross(elem.second.force_value);
        
        EXPECT_NEAR(elem.second.position_value.z(), ground_z, 1e-6);
        EXPECT_NEAR(elem.second.normal_value.norm(), 1.0, 1e-6);
        EXPECT_NEAR(elem.second.normal_value.z(), 1.0, 1e-6);
        
        Eigen::Vector3d F_tmp, n_tmp;
        F_tmp = elem.second.force_value;
        n_tmp = elem.second.normal_value;
        
        EXPECT_TRUE((-F_tmp.dot(n_tmp)) <= 0.0);
        EXPECT_TRUE(((F_tmp-(n_tmp.dot(F_tmp))*n_tmp).norm() - mu*(F_tmp.dot(n_tmp))) <= 0.0); 
        
    }
    
    EXPECT_NEAR(F_sum.x(), manip_wrench[0], 1e-6);
    EXPECT_NEAR(F_sum.y(), manip_wrench[1], 1e-6);
    EXPECT_NEAR(F_sum.z(), -robot_mass*g + manip_wrench[2], 1e-6);
    EXPECT_NEAR(Torque_sum.x(), manip_wrench[3], 1e-5);
    EXPECT_NEAR(Torque_sum.y(), manip_wrench[4], 1e-5);
    EXPECT_NEAR(Torque_sum.z(), manip_wrench[5], 1e-5);
    
    
};


TEST_F(TestBasic, testSuperquadricEnv)
{
    
    double robot_mass = 100.0;
    double g = -9.81;
    
    std::vector<std::string> contact_name;
    contact_name.push_back("contact1");
    contact_name.push_back("contact2");
    contact_name.push_back("contact3");
    contact_name.push_back("contact4");
    
    auto superquadric_env = std::make_shared<cpl::env::Superquadric>();
    double mu = 0.5;
    superquadric_env->SetMu(mu);
    Eigen::Vector3d C, R, P;
    C << 0.0, 0.0, 1.0;
    R << 0.3, 0.3, 10.0;
    P << 10.0, 10.0, 10.0;
    superquadric_env->SetParameters(C,R,P);
    
    auto cpl = std::make_shared<cpl::CentroidalPlanner>(contact_name, robot_mass, superquadric_env);
    cpl->SetForceWeight(0.0);
    
    Eigen::Vector3d p_lb, p_ub;
    p_lb  << -0.5, -0.5, 0.5;
    p_ub  <<  0.5,  0.5, 1.5;

    cpl->SetPosBounds("contact1",p_lb,p_ub);
    cpl->SetPosBounds("contact2",p_lb,p_ub);
    cpl->SetPosBounds("contact3",p_lb,p_ub);
    cpl->SetPosBounds("contact4",p_lb,p_ub);
    
    Eigen::VectorXd manip_wrench;
    manip_wrench.setZero(6);
    manip_wrench[0] = 100.0;
    manip_wrench[5] = 100.0;
    cpl->SetManipulationWrench(manip_wrench);
    
    cpl::solver::Solution sol;
    int status = cpl->Solve(sol);

    std::cout<< "sol:" << sol << std::endl;
    std::cout<< "status:" << status << std::endl;

    Eigen::Vector3d F_sum, Torque_sum; 
    F_sum.setZero();
    Torque_sum.setZero();
    
    double superquadric = 0.0;
    
    for (auto& elem: sol.contact_values_map)
    {       
        F_sum += elem.second.force_value;         
        Torque_sum += (elem.second.position_value - sol.com_sol).cross(elem.second.force_value);
        
        superquadric = pow((elem.second.position_value.x()-C.x())/R.x(),P.x())+ 
                       pow((elem.second.position_value.y()-C.y())/R.y(),P.y())+ 
                       pow((elem.second.position_value.z()-C.z())/R.z(),P.z());
                       
        EXPECT_NEAR(superquadric, 1.0, 1e-4);               
        EXPECT_NEAR(elem.second.normal_value.norm(), 1.0, 1e-6);               
        
        Eigen::Vector3d F_tmp, n_tmp;
        F_tmp = elem.second.force_value;
        n_tmp = elem.second.normal_value;
        
        EXPECT_TRUE((-F_tmp.dot(n_tmp)) <= 0.0);
        EXPECT_TRUE(((F_tmp-(n_tmp.dot(F_tmp))*n_tmp).norm() - mu*(F_tmp.dot(n_tmp))) <= 0.0); 
                
        EXPECT_TRUE((elem.second.position_value.x() - p_lb.x()) >= 0.0);
        EXPECT_TRUE((elem.second.position_value.x() - p_ub.x()) <= 0.0); 
        EXPECT_TRUE((elem.second.position_value.y() - p_lb.y()) >= 0.0);
        EXPECT_TRUE((elem.second.position_value.y() - p_ub.y()) <= 0.0); 
        EXPECT_TRUE((elem.second.position_value.z() - p_lb.z()) >= 0.0);
        EXPECT_TRUE((elem.second.position_value.z() - p_ub.z()) <= 0.0); 
        
    }
    
    EXPECT_NEAR(F_sum.x(), manip_wrench[0], 1e-6);
    EXPECT_NEAR(F_sum.y(), manip_wrench[1], 1e-6);
    EXPECT_NEAR(F_sum.z(), -robot_mass*g + manip_wrench[2], 1e-6);
    EXPECT_NEAR(Torque_sum.x(), manip_wrench[3], 1e-4);
    EXPECT_NEAR(Torque_sum.y(), manip_wrench[4], 1e-4);
    EXPECT_NEAR(Torque_sum.z(), manip_wrench[5], 1e-4);
    
};


TEST_F(TestBasic, testCoMPlanner)
{
    
    double robot_mass = 100.0;
    double g = -9.81;
    
    std::vector<std::string> contact_name;
    contact_name.push_back("contact1");
    contact_name.push_back("contact2");
    contact_name.push_back("contact3");
    contact_name.push_back("contact4");
    
    auto cpl = std::make_shared<cpl::CoMPlanner>(contact_name, robot_mass);
    
    double mu = 0.5;
    cpl->SetMu(mu);
    
    std::cout <<"mu: "<< cpl->GetMu() << std::endl;
    
    cpl->SetContactPosition("contact1", Eigen::Vector3d(1.0, 1.0, 0.0));
    cpl->SetContactPosition("contact2", Eigen::Vector3d(-1.0, 1.0, 0.0));
    cpl->SetContactPosition("contact3", Eigen::Vector3d(-1.0, -1.0, 0.0));
    cpl->SetContactPosition("contact4", Eigen::Vector3d(1.0, -1.0, 0.0));
    
    cpl->SetLiftingContact("contact4");
    
    auto lifting_contacts = cpl->GetLiftingContacts();
    
    for (auto& l : lifting_contacts)
    {
        std::cout << "Lifting: " << l << std::endl;
    }
  
    for(auto c : contact_name)
    {
        cpl->SetForceThreshold(c, 20.0);
    }
    
    cpl::solver::Solution sol;
    int status = cpl->Solve(sol);

    std::cout<< "sol:" << sol << std::endl;
    std::cout<< "status:" << status << std::endl;

    Eigen::Vector3d F_sum, Torque_sum; 
    F_sum.setZero();
    Torque_sum.setZero();
    
    for (auto& elem: sol.contact_values_map)
    {       
        F_sum += elem.second.force_value;         
        Torque_sum += (elem.second.position_value - sol.com_sol).cross(elem.second.force_value);             
        
        Eigen::Vector3d F, n;
        F = elem.second.force_value;
        n = elem.second.normal_value;
       
        EXPECT_TRUE((-F.dot(n)) <= 0.0);
        EXPECT_LE((F-(n.dot(F))*n).norm() - mu*(F.dot(n)), 0.0);
        
    }
    
    EXPECT_NEAR(F_sum.x(), 0.0, 1e-6);
    EXPECT_NEAR(F_sum.y(), 0.0, 1e-6);
    EXPECT_NEAR(F_sum.z(), -robot_mass*g, 1e-6);
    EXPECT_NEAR(Torque_sum.x(), 0.0, 1e-4);
    EXPECT_NEAR(Torque_sum.y(), 0.0, 1e-4);
    EXPECT_NEAR(Torque_sum.z(), 0.0, 1e-4);
    
};

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
