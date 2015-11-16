#include "jsk_topic_tools/rosparam_utils.h"
#include <XmlRpcValue.h>
#include <gtest/gtest.h>


TEST(RoparamUtils, testGetXmlDoubleValue){
  // type int
  XmlRpc::XmlRpcValue v = 1;
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeInt, v.getType());
  EXPECT_EQ(1.0, jsk_topic_tools::getXMLDoubleValue(v));
  // type double
  v = 1.0;
  EXPECT_EQ(XmlRpc::XmlRpcValue::TypeDouble, v.getType());
  EXPECT_EQ(1.0, jsk_topic_tools::getXMLDoubleValue(v));
  // test unparsable param
  try {
    v = "-.3";
    EXPECT_EQ(-0.3, jsk_topic_tools::getXMLDoubleValue(v));
  } catch (std::runtime_error &e) {
    SUCCEED() << "Not parsable value";
    return;
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
