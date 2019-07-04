#include <gtest/gtest.h>
#include <CentroidalPlanner/CentroidalPlanner.h>

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
    
    auto sol = cpl->Solve();
    
    std::cout << sol << std::endl;
    
    double Fz_tot = 0.0;
    
    for (auto& elem: sol.contact_values_map)
    {       
        Fz_tot += elem.second.force_value.z();  
        
        EXPECT_NEAR(elem.second.position_value.z(), ground_z, 1e-12);
        EXPECT_NEAR(elem.second.normal_value.norm(), 1.0, 1e-12);
        EXPECT_NEAR(elem.second.normal_value.z(), 1.0, 1e-12);
        
    }

    EXPECT_NEAR(Fz_tot, -robot_mass*g, 1e-12);
    
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
    ground_env->SetMu(0.5);
    
    auto cpl = std::make_shared<cpl::CentroidalPlanner>(contact_name, robot_mass, ground_env);
    cpl->SetCoMWeight(2.0);
    
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
    
    auto sol = cpl->Solve();
    
    std::cout << sol << std::endl;
    
    Eigen::Vector3d F_sum, Torque_sum; 
    F_sum.setZero();
    Torque_sum.setZero();
    
    for (auto& elem: sol.contact_values_map)
    {       
        F_sum += elem.second.force_value;   
        Torque_sum += (elem.second.position_value - sol.com_sol).cross(elem.second.force_value);
        
        EXPECT_NEAR(elem.second.position_value.z(), ground_z, 1e-12);
        EXPECT_NEAR(elem.second.normal_value.norm(), 1.0, 1e-12);
        EXPECT_NEAR(elem.second.normal_value.z(), 1.0, 1e-12);
        
    }
    
    EXPECT_NEAR(F_sum.x(), manip_wrench[0], 1e-12);
    EXPECT_NEAR(F_sum.y(), manip_wrench[1], 1e-12);
    EXPECT_NEAR(F_sum.z(), -robot_mass*g + manip_wrench[2], 1e-12);
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
    superquadric_env->SetMu(0.5);
    Eigen::Vector3d C, R, P;
    C << 0.0, 0.0, 1.0;
    R << 0.3, 0.3, 10.0;
    P << 10.0, 10.0, 10.0;
    superquadric_env->SetParameters(C,R,P);
    
    auto cpl = std::make_shared<cpl::CentroidalPlanner>(contact_name, robot_mass, superquadric_env);
    cpl->SetCoMWeight(1.0);
    cpl->SetPosWeight(1.0);
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
    
    auto sol = cpl->Solve();
    
    std::cout << sol << std::endl;
    
    Eigen::Vector3d F_sum, Torque_sum; 
    F_sum.setZero();
    Torque_sum.setZero();
    
    for (auto& elem: sol.contact_values_map)
    {       
        F_sum += elem.second.force_value;         
        Torque_sum += (elem.second.position_value - sol.com_sol).cross(elem.second.force_value);
        
        EXPECT_NEAR(elem.second.normal_value.norm(), 1.0, 1e-12);
        
    }
    
    EXPECT_NEAR(F_sum.x(), manip_wrench[0], 1e-12);
    EXPECT_NEAR(F_sum.y(), manip_wrench[1], 1e-12);
    EXPECT_NEAR(F_sum.z(), -robot_mass*g + manip_wrench[2], 1e-12);
    EXPECT_NEAR(Torque_sum.x(), manip_wrench[3], 1e-4);
    EXPECT_NEAR(Torque_sum.y(), manip_wrench[4], 1e-4);
    EXPECT_NEAR(Torque_sum.z(), manip_wrench[5], 1e-4);
    
};


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}