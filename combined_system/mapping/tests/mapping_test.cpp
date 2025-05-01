#include <gtest/gtest.h>
#include <mapping.h>

class MappingTest : public ::testing::Test {
    protected:
        
        MappingProcess map;
    
        void SetUp() override {
            ros::NodeHandle nh("~");
            map.init(nh);
        }
    };

TEST_F(MappingTest, IsInMap2D) {
    Eigen::Vector2d pos(0.0, 0.0);
    EXPECT_TRUE(map.isInMap2D(pos));
}

TEST_F(MappingTest, IsNotInMap2D) {
    Eigen::Vector2d pos(100.0, 100.0);
    EXPECT_FALSE(map.isInMap2D(pos));
}

TEST_F(MappingTest, PosToIndex2D) {
    Eigen::Vector2d pos(0.0, 0.0);
    int index;
    map.posToIndex2d(pos, index);
    EXPECT_EQ(index, 0);
}

TEST_F(MappingTest, IndexToPos2D) {
    Eigen::Vector2i index(0, 0);
    Eigen::Vector2d pos;
    map.indexToPos2d(index, pos);
    EXPECT_NEAR(pos(0), 0.0, 1e-6);
    EXPECT_NEAR(pos(1), 0.0, 1e-6);
}

TEST_F(MappingTest, GetVoxelState2D) {
    Eigen::Vector2d pos(0.0, 0.0);
    int state = map.getVoxelState2d(pos);
    EXPECT_EQ(state, 0); 
}

TEST_F(MappingTest, SetObstacle) {
    Eigen::Vector2d pos(0.0, 0.0);
    map.setObstacle(pos);
    int state = map.getVoxelState2d(pos);
    EXPECT_EQ(state, 1); 
}