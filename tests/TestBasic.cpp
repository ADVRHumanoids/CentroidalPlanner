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

TEST_F(TestBasic, check1)
{
    
    double robot_mass = 100;
    
    std::vector<std::string> contact_name;
    contact_name.push_back("contact1");
    contact_name.push_back("contact2");
    contact_name.push_back("contact3");
    contact_name.push_back("contact4");
    
    auto superquadric_env = std::make_shared<cpl::env::Superquadric>();
    
    auto cpl_superquadric = std::make_shared<cpl::CentroidalPlanner>(contact_name, robot_mass, superquadric_env);   
    auto sol1 = cpl_superquadric->Solve();
    
    std::cout << sol1 << std::endl;
    
    auto ground_env = std::make_shared<cpl::env::Ground>();
    ground_env->SetGroundZ(0.1);
    ground_env->SetMu(0.5);
    
    auto cpl_ground = std::make_shared<cpl::CentroidalPlanner>(contact_name, robot_mass, ground_env);
    auto sol2 = cpl_ground->Solve();
    
    std::cout << sol2 << std::endl;
    
};


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}