#include <caros/trakstar_node.h>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "caros_trakstar_node");

   if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
   }

   caros::TrakstarNode tstar;
   std::map<std::string,std::string> mstring;
   std::vector<std::string> vstring;
   tstar.init(std::string("trakstarnode"),mstring,vstring);

   tstar.start();

   return 0;
}
